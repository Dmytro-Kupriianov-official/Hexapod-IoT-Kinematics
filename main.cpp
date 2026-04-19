#include <Arduino.h>
#include <math.h>

/*** Wi-Fi + Web ***/
#if defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  ESP8266WebServer server(80);
#elif defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  WebServer server(80);
#else
  #error "This sketch needs ESP8266 or ESP32."
#endif

// --- Twój hotspot (tryb STA) ---
const char* WIFI_SSID = "Mac";
const char* WIFI_PASS = "asdf1107";

// --- Przełącznik chodu (gait) ON/OFF sterowany z WWW ---
volatile bool gaitEnabled = true;

/*** GEOMETRIA NOGI ***/
// Długości segmentów i bazowe przesunięcia pozy spoczynkowej.
// J2L/J3L: długości femur i tibia (mm). Y_Rest/Z_Rest: offset pozy spoczynku.
// J3_LegAngle: kalibracja tibii (korekta geometryczna w stopniach).
const double J2L = 70.0;        // femur [mm]
const double J3L = 120.0;       // tibia [mm]
const double Y_Rest = 70.0;     // bazowe odsunięcie w osi Y (poza spocz.)
const double Z_Rest = -80.0;    // bazowy poziom w osi Z (poza spocz.)
const double J3_LegAngle = 15.4;// kalibracja tibii [deg]

/*** ORIENTACJE/ZNaki DLA SERW ***/
// Kierunki przeliczeń kątów -> serwomechanizmów.
// Nie zmieniać: te znaki odpowiadają Twojej mechanice i okablowaniu.
const int8_t DIR_COXA  = -1;            // inwersja J1 (coxa)
const bool   FEMUR_FLIPPED_180 = true;  // femur obrócony o 180°
const int8_t DIR_TIBIA = +1;            // tibia jak u autora

/*** PARAMETRY SERWOMECHANIZMÓW ***/
// Zakres impulsów PWM w mikrosekundach. Mapowanie 0..180° -> 500..2500 µs.
const int PWM_MIN = 500, PWM_MAX = 2500;

/*** PARAMETRY CHODU (TRIPOD) ***/
// STEP_HEIGHT: wysokość kroku (amplituda unoszenia stopy).
// STEP_LEN: długość kroku (połówka: ±STEP_LEN/2 w fazach podporu/przenosu).
// Y_BASE: bazowe Y (odsunięcie boczne stopy względem środka ciała).
// Z_GROUND: poziom „podłoża” (wartości ujemne = poniżej środka ciała).
// DUTY: udział fazy podporu (0..1). PERIOD_MS: czas całego cyklu kroku.
// FRAME_MS: odstęp między klatkami aktualizacji (sterowanie ~33 Hz).
volatile float STEP_HEIGHT = 25.0f;   // [mm]
volatile float STEP_LEN    = 60.0f;   // [mm]
volatile float Y_BASE      = 20.0f;   // [mm] (zmieniane z WWW)
volatile float Z_GROUND    = -50.0f;  // [mm] (zmieniane z WWW)
volatile float DUTY        = 0.6f;    // [–]  (zmieniane z WWW)
const uint16_t PERIOD_MS   = 1200;    // [ms]
const uint16_t FRAME_MS    = 30;      // [ms]

uint32_t t0 = 0, tPrev = 0;

/*** NARZĘDZIA POMOCNICZE ***/
static inline double clampd(double v, double lo, double hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
static inline double clampDeg(double a){ if(a<0) a=0; if(a>180) a=180; return a; }
static inline long   degToUs(double ang){
  // Mapowanie kąta w stopniach (0..180) na szerokość impulsu PWM (µs)
  long us = (long)((ang/180.0)*2000.0 + 500.0);
  if(us<PWM_MIN) us=PWM_MIN;
  if(us>PWM_MAX) us=PWM_MAX;
  return us;
}

/*** Wysyłka ramek do sterownika Veyron ***/
// Wysyła jedną „paczkę” trzech kanałów: coxa, femur, tibia + czas ruchu Tms.
// Format: "#<ch> P<us> #<ch> P<us> #<ch> P<us> T<ms>\r"
void sendPulsesTo(uint8_t ch_coxa, uint8_t ch_femur, uint8_t ch_tibia,
                  long us_coxa, long us_femur, long us_tibia, uint16_t Tms){
  Serial.print("#"); Serial.print(ch_coxa);  Serial.print(" P"); Serial.print(us_coxa);
  Serial.print(" #"); Serial.print(ch_femur); Serial.print(" P"); Serial.print(us_femur);
  Serial.print(" #"); Serial.print(ch_tibia); Serial.print(" P"); Serial.print(us_tibia);
  Serial.print(" T"); Serial.print(Tms);
  Serial.print("\r");
}

/*** TRAJEKTORIA STOPY (TRIPOD) ***/
// stride1D: generuje ruch wzdłuż osi kroku S (piłokształtny przebieg z łukiem unoszenia).
// phi: faza 0..1, S: pełna długość kroku, Spos: wyjście (pozycja wzdłuż S), Z: wysokość.
struct StepXY { float dX; float dY; float dZ; }; // lokalne przesunięcia względem bazy

static inline void stride1D(float phi, float S, float& Spos, float& Z){
  if(phi < DUTY){
    // Faza podporu: stopa „toczy się” po podłożu z +S/2 do −S/2
    float s = phi / DUTY;
    Spos = +0.5f*S - S*s;
    Z = Z_GROUND;
  } else {
    // Faza przenosu: stopa wraca z −S/2 do +S/2 łukiem (sinus) nad podłożem
    float s = (phi - DUTY) / (1.0f - DUTY);
    Spos = -0.5f*S + S*s;
    Z = Z_GROUND + STEP_HEIGHT * sinf(PI * s);
  }
}

/*** OPIS KANAŁÓW NÓG (KOLEJNOŚĆ) ***/
// Porządek: Lr, Rr, Rm, Lm, Lf, Rf (tylna lewa, tylna prawa, środkowa prawa, środkowa lewa, przednia lewa, przednia prawa)
struct Leg { uint8_t coxa, femur, tibia; float phase; };
Leg legs[6] = {
  { 13, 14, 12, 0.0f },   // Lr
  {  9, 10, 11, 0.0f },   // Rr
  {  6,  5,  7, 0.0f },   // Rm
  { 17, 18, 16, 0.0f },   // Lm
  { 22, 21, 20, 0.0f },   // Lf
  {  1,  2,  0, 0.0f },   // Rf
};

// Wygodne aliasy indeksów
enum LegID { Lr=0, Rr, Rm, Lm, Lf, Rf, LEG_COUNT };

/*** KIERUNKI KROKU PO OSIACH ***/
// stepDirX: kierunek posuwu wzdłuż osi X dla każdej nogi (zachowane jak w Twoim kodzie).
// stepDirY: kierunek strafe’u (bocznego) w osi Y (naprzemiennie lewa/prawa).
int8_t stepDirX[LEG_COUNT] = { -1, +1, -1, -1, -1, +1 };
int8_t stepDirY[LEG_COUNT] = { +1, -1, +1, -1, +1, -1 };

// Kierunki dla coxa/femur/tibia – nie zmieniać (dopasowane do mechaniki i okablowania)
int8_t dirTibia[LEG_COUNT] = { +1, -1, -1, +1, +1, -1 };
int8_t dirFemur[LEG_COUNT] = { +1, -1, -1, +1, +1, -1 };
int8_t dirCoxa [LEG_COUNT] = { +1, -1, -1, +1, +1, +1 };

/*** KĄTOWE OFFSETY MIĘDZY NOGAMI (KALIBRACJA) ***/
// Pozwalają skompensować różnice montażowe serw i dźwigni.
// Uwaga: wartości w stopniach, dodatnie/ujemne zgodnie z mapowaniem powyżej.
double coxaOff [LEG_COUNT] = {0,0,0,0,0,0};
double femurOff[LEG_COUNT] = {0,0,0,0,0,0};
double tibiaOff[LEG_COUNT] = {-60,60,60,-60,-60,60};

/*** FAZY TRIPODU ***/
// Dwie trójki nóg pracują w przeciwfazie: A = {Lf, Rm, Lr} (phi=0.0), B = {Rf, Lm, Rr} (phi=0.5).
void setPhases(){
  legs[Lf].phase = 0.0f;
  legs[Rm].phase = 0.0f;
  legs[Lr].phase = 0.0f;

  legs[Rf].phase = 0.5f;
  legs[Lm].phase = 0.5f;
  legs[Rr].phase = 0.5f;
}

/*** IK + offsety + wysyłka dla jednej nogi ***/
// Liczy kąty J1/J2/J3 z (X,Y,Z), zamienia na kąty serw z uwzględnieniem znaków i offsetów,
// tnie do 0..180° i wysyła jako impulsy µs do odpowiednich kanałów Veyrona.
void moveXYZ_leg(int idx, double X, double Y, double Z, uint16_t Tms){
  // Poza bazowa (dodajemy spoczynkowe offsety)
  Y += Y_Rest;
  Z += Z_Rest;

  // IK (bez zmian względem Twojej wersji)
  double J1 = atan2(X, Y) * 180.0/PI;
  double H  = sqrt(Y*Y + X*X);
  double L  = sqrt(H*H + Z*Z);

  double c3 = clampd((J2L*J2L + J3L*J3L - L*L) / (2.0*J2L*J3L), -1.0, 1.0);
  double J3 = acos(c3) * 180.0/PI;

  double cb = clampd((L*L + J2L*J2L - J3L*J3L) / (2.0*L*J2L), -1.0, 1.0);
  double B  = acos(cb) * 180.0/PI;

  double A  = atan2(Z, H) * 180.0/PI;
  double J2 = B + A;

  // Mapowanie na kąty serw (zachowane znaki/kierunki dokładnie jak w oryginale)
  double coxa_servo  = 90 - (dirCoxa[idx] * J1);
  double femur_servo = FEMUR_FLIPPED_180 ? (90 + dirFemur[idx]*J2) : (90 - dirFemur[idx]*J2);
  double tibia_servo = 90 + dirTibia[idx] * (J3 + J3_LegAngle - 90);

  // Offsety kalibracyjne
  coxa_servo  += coxaOff[idx];
  femur_servo += femurOff[idx];
  tibia_servo += tibiaOff[idx];

  // Ograniczenie do zakresu i konwersja na µs
  coxa_servo  = clampDeg(coxa_servo);
  femur_servo = clampDeg(femur_servo);
  tibia_servo = clampDeg(tibia_servo);

  long us_c = degToUs(coxa_servo);
  long us_f = degToUs(femur_servo);
  long us_t = degToUs(tibia_servo);

  sendPulsesTo(legs[idx].coxa, legs[idx].femur, legs[idx].tibia, us_c, us_f, us_t, Tms);
}

/*** Ustawienie wszystkich nóg w pozie neutralnej ***/
void setNeutralAll(double X, double Y, double Z, uint16_t Tms){
  for(int i=0;i<LEG_COUNT;i++) moveXYZ_leg(i, X, Y, Z, Tms);
}

/*** TRYB RUCHU ***/
// MOVE_FWD: krok do przodu po osi X (jak wcześniej).
// MOVE_LEFT/RIGHT: strafe boczny po osi Y (używa stepDirY i Y_BASE).
enum MoveMode { MOVE_FWD = 0, MOVE_LEFT = 1, MOVE_RIGHT = 2 };
volatile MoveMode moveMode = MOVE_FWD;

/*** STRONA WWW (UI) ***/
// Zawiera przyciski: ON/OFF chodu, wybór kierunku (Forward/Left/Right),
// formularze live-zmian parametrów (STEP_HEIGHT/STEP_LEN/Y_BASE/Z_GROUND/DUTY),
// oraz pionowy suwak do analogowej zmiany STEP_LEN (zakres −160..+160).
String htmlPage(){
  String btn = gaitEnabled ? "Gait: ON \xE2\x86\x92 OFF" : "Gait: OFF \xE2\x86\x92 ON";
  String state = gaitEnabled ? "ON" : "OFF";

  String md;
  if (moveMode == MOVE_FWD)  md = "Forward";
  else if (moveMode == MOVE_LEFT) md = "Left (strafe)";
  else md = "Right (strafe)";

  String page;
  page.reserve(7000);
  page += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Hexapod Leg</title>"
            "<style>"
            "body{font-family:system-ui;margin:16px;max-width:640px}"
            "input[type=number]{width:100%;padding:8px;margin:6px 0}"
            "button{width:100%;padding:10px;margin:6px 0}"
            ".row{display:flex;gap:8px}.row>*{flex:1}"
            ".panel{border:1px solid #ddd;border-radius:12px;padding:12px;margin:10px 0;box-shadow:0 1px 4px rgba(0,0,0,.05)}"
            ".vwrap{display:flex;gap:14px;align-items:center;justify-content:space-between}"
            ".vslider{height:220px;width:48px;writing-mode:bt-lr;-webkit-appearance:slider-vertical;}"
            ".hint{font-size:12px;color:#666}"
            "</style>"
            "</head><body>");
  page += F("<h2>Hexapod: Leg Control</h2>");
  page += F("<p><b>Gait:</b> "); page += state; page += F("</p>");
  page += F("<p><b>Mode:</b> "); page += md; page += F("</p>");

  // Przełącznik chodu + wybór kierunku ruchu
  page += F("<div class='panel'>"
              "<form action='/set' method='GET'><button name='toggle' value='1' type='submit'>");
  page += btn;
  page += F("</button></form>"
            "<div class='row'>"
              "<form action='/set' method='GET'><button name='mode' value='fwd' type='submit'>Forward</button></form>"
              "<form action='/set' method='GET'><button name='mode' value='left' type='submit'>Left</button></form>"
              "<form action='/set' method='GET'><button name='mode' value='right' type='submit'>Right</button></form>"
            "</div></div>");

  // Parametry chodu (zmiana w locie)
  page += F("<div class='panel'><h3>Gait Params</h3>");

  page += F("<form action='/set' method='GET'>"
            "<label>STEP_HEIGHT (mm):</label>"
            "<input name='step' type='number' step='1' value='");
  page += String(STEP_HEIGHT, 1);
  page += F("'><button type='submit'>Set STEP_HEIGHT</button></form>");

  page += F("<form action='/set' method='GET'>"
            "<label>STEP_LEN (mm):</label>"
            "<input name='steplen' type='number' step='1' value='");
  page += String(STEP_LEN, 1);
  page += F("'><button type='submit'>Set STEP_LEN</button></form>");

  page += F("<form action='/set' method='GET'>"
            "<label>Y_BASE (mm):</label>"
            "<input name='ybase' type='number' step='1' value='");
  page += String(Y_BASE, 1);
  page += F("'><button type='submit'>Set Y_BASE</button></form>");

  page += F("<form action='/set' method='GET'>"
            "<label>Z_GROUND (mm):</label>"
            "<input name='zground' type='number' step='1' value='");
  page += String(Z_GROUND, 1);
  page += F("'><button type='submit'>Set Z_GROUND</button></form>");

  page += F("<form action='/set' method='GET'>"
            "<label>DUTY (0..1):</label>"
            "<input name='duty' type='number' step='0.01' min='0.05' max='0.95' value='");
  page += String(DUTY, 2);
  page += F("'><button type='submit'>Set DUTY</button></form></div>");

  // Pionowy suwak do analogowej zmiany STEP_LEN (−160..+160)
  page += F("<div class='panel'><h3>Analog STEP_LEN</h3>"
            "<div class='vwrap'>"
              "<div class='col'>"
                "<p class='hint'>Suwak wysyła <code>steplen</code> co ~80 ms. Zakres −160…+160 (dodatkowe, szybkie sterowanie długością kroku).</p>"
              "</div>"
              "<div>"
                "<input class='vslider' id='stepLenSlider' type='range' min='-160' max='160' value='0' orient='vertical'>"
                "<div style='text-align:center;margin-top:6px' id='slv'>0</div>"
              "</div>"
            "</div>"
            "</div>");

  // JavaScript dla suwaka (odświeżanie z niewielkim opóźnieniem, żeby nie zalać /set)
  page += R"rawliteral(
<script>
let sendTimer = null;
function sendStepLen(val){
  const p = new URLSearchParams({ steplen: String(val) });
  fetch(`/set?${p}`).catch(()=>{});
}
const slider = document.getElementById('stepLenSlider');
const slv = document.getElementById('slv');
if (slider){
  slider.addEventListener('input', () => {
    slv.textContent = slider.value;
    if (sendTimer) clearTimeout(sendTimer);
    sendTimer = setTimeout(() => sendStepLen(slider.value), 80);
  });
}
</script>
)rawliteral";

  page += F("<hr><small>Open Serial @115200 (Both NL & CR). IP: ");
  page += WiFi.localIP().toString();
  page += F("</small></body></html>");
  return page;
}

/*** Handlery HTTP ***/
// GET /           -> generuje stronę HTML z UI
// GET /set?...    -> przyjmuje parametry (toggle, step, steplen, ybase, zground, duty, mode) i aktualizuje zmienne
void handleRoot(){ server.send(200, "text/html", htmlPage()); }

void handleSet(){
  if(server.hasArg("toggle"))   gaitEnabled = !gaitEnabled;
  if(server.hasArg("step"))     STEP_HEIGHT = server.arg("step").toFloat();
  if(server.hasArg("steplen"))  STEP_LEN    = server.arg("steplen").toFloat();
  if(server.hasArg("ybase"))    Y_BASE      = server.arg("ybase").toFloat();
  if(server.hasArg("zground"))  Z_GROUND    = server.arg("zground").toFloat();
  if(server.hasArg("duty")){
    float d = server.arg("duty").toFloat();
    // Bezpieczne ograniczenie: unikamy skrajnych wartości fazy podporu
    if(d < 0.05f) d = 0.05f;
    if(d > 0.95f) d = 0.95f;
    DUTY = d;
  }
  if(server.hasArg("mode")){
    String m = server.arg("mode");
    if(m == "fwd")   moveMode = MOVE_FWD;
    if(m == "left")  moveMode = MOVE_LEFT;
    if(m == "right") moveMode = MOVE_RIGHT;
  }
  // Przekierowanie z powrotem na stronę główną po zmianie
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "OK");
}

/*** SETUP / LOOP ***/
// Inicjalizacja Wi-Fi i serwera WWW, ustawienie faz tripod,
// opcjonalne wstawienie pozy startowej, pętla: obsługa WWW + generacja kroków.
void setup(){
  Serial.begin(115200);
  t0 = millis();
  Serial.println(F("Tripod gait with strafe (UI slider + live params). Veyron @115200, Both NL & CR."));

#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.persistent(false);
  WiFi.setSleep(false);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print(F("WiFi: connecting"));
  uint32_t tstart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - tstart < 15000) {
    delay(250);
    Serial.print(".");
#if defined(ARDUINO_ARCH_ESP8266)
    yield();
#endif
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("WiFi connected. IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi connect TIMEOUT (check SSID/PASS)."));
  }

  setPhases();                   // stałe fazy tripod
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();

  // Poza startowa (opcjonalnie, łagodny najazd)
  setNeutralAll(0, 20, -50, 700);
}

void loop(){
  server.handleClient();
#if defined(ARDUINO_ARCH_ESP8266)
  yield();
#endif

  uint32_t now = millis();
  if (now - tPrev < FRAME_MS) return;
  tPrev = now;

  if (gaitEnabled) {
    // Faza bazowa 0..1 z okresu PERIOD_MS
    float basePhi = fmodf((now - t0) / (float)PERIOD_MS, 1.0f);

    for (int i = 0; i < LEG_COUNT; i++) {
      // Przesunięcie fazy dla danej nogi (A=0.0, B=0.5)
      float phi = basePhi + legs[i].phase;
      phi = fmodf(phi, 1.0f);

      // Ruch 1D wzdłuż osi kroku + wysokość
      float Spos, Z;
      stride1D(phi, STEP_LEN, Spos, Z);

      // Składanie ruchu po osiach:
      // - Forward: posuw po X (stepDirX), Y = stałe Y_BASE
      // - Strafe:  posuw po Y (stepDirY), X = 0
      float dX = 0.0f, dY = 0.0f;
      if (moveMode == MOVE_FWD) {
        dX = Spos * stepDirX[i];
        dY = Y_BASE;
      } else if (moveMode == MOVE_LEFT) {
        dX = 0.0f;
        dY = Y_BASE + (Spos * stepDirY[i]);
      } else { // MOVE_RIGHT
        dX = 0.0f;
        dY = Y_BASE - (Spos * stepDirY[i]);
      }

      // IK + wysyłka dla bieżącej nogi
      moveXYZ_leg(i, dX, dY, Z, FRAME_MS);
    }
  }
}
