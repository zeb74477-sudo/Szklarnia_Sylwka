
/**************************************************************

 *  PROJEKT : Sterownik Szklarni / Namiotu Uprawowego
 *  AUTOR   : ALEA – Wspaniała  &  zeb'ER
 *            27-12-2025
 *  OPIS:
 *  Autonomiczny sterownik wilgotności, wentylacji i zabezpieczeń
 *  z odpornym systemem pamięci (FLASH + GOLD + DEFAULT),
 *  obsługą alarmów, watchdogiem i interfejsem OLED.
 *
 *  FILOZOFIA:
 *  - System MUSI działać nawet przy awarii pamięci
 *  - Brak delay(), tylko millis()
 *  - BOOT zawsze kończy się RUN
 *  - Jedno źródło prawdy: active_config
 *
 *  WERSJA:
 *  v0.9 – szkielet funkcjonalny (bez strojenia)
 **************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>

// --- DHT ---
#define DHTPIN 2          // pin danych DHT (zmień jeśli inny)
#define DHTTYPE DHT22     // typ czujnika

DHT dht(DHTPIN, DHTTYPE);

/* ============================================================
 *                      WERSJA SYSTEMU
 * ============================================================
 */

#define FW_VERSION_MAJOR   0
#define FW_VERSION_MINOR   9

/* ============================================================
 *                      PINMAP (HARDWARE)
 * ============================================================
 *  Zdefiniowane centralnie – łatwa migracja NANO / PRO MINI
 * ============================================================
 */

// --- Przekaźniki ---
#define PIN_FAN_RELAY        3   // wentylator
#define PIN_HUM_RELAY        4   // nawilżacze
#define PIN_PUMP_RELAY       8   // pompa wody

// --- Enkoder / przyciski ---
#define PIN_ENC_A            5
#define PIN_ENC_B            6
#define PIN_ENC_BTN          7

// --- Dioda alarmowa ---
#define PIN_LED_ERROR        13

// --- SPI FLASH ---
#define PIN_FLASH_CS         10  // Chip Select pamięci W25Qxx

// --- Czujniki poziomu wody ---
#define PIN_WATER_LOW_MAIN   A0
#define PIN_WATER_LOW_HUM    A1
#define PIN_WATER_HIGH_HUM   A2

/* ============================================================
 *                  MAGIA, WERSJE, STAŁE
 * ============================================================
 */

// Magiczne identyfikatory rekordów
#define RECORD_MAGIC_CONFIG  0x4346   // 'CF'
#define RECORD_MAGIC_STAT    0x5354   // 'ST'

// Wersje struktur
#define CONFIG_RECORD_VERSION  1
#define STAT_RECORD_VERSION    1

/* ============================================================
 *                  MAPA PAMIĘCI FLASH
 * ============================================================
 */

#define FLASH_PAGE_SIZE        4096UL   // 4kB

// --- CONFIG ---
#define FLASH_CONFIG_START     0x000000UL
#define FLASH_CONFIG_PAGES     8         // 32kB na konfigurację (rotacja)

// --- STAT / LOG ---
#define FLASH_STAT_START       0x008000UL
#define FLASH_STAT_PAGES       16        // 64kB na statystyki

// --- STREFA OCHRONNA ---
#define FLASH_GUARD_PAGES      4         // bufor bezpieczeństwa

/* ============================================================
 *                  FLAGI AWARII (BITMASKA)
 * ============================================================
 */

#define FAULT_SPI_FLASH       0x01  // awaria pamięci
#define FAULT_WDT_RESET       0x02  // reset watchdog
#define FAULT_SENSOR_ERROR    0x04  // błąd czujnika
#define FAULT_PUMP_TIMEOUT    0x08  // timeout pompy

/* ============================================================
 *              STANY SYSTEMU (FSM)
 * ============================================================
 */

enum SystemState
{
  STATE_BOOT = 0,   // start systemu
  STATE_RUN,        // normalna praca
  STATE_ALARM,      // aktywny alarm
  STATE_SLEEP       // ekran wygaszony
};

SystemState system_state = STATE_BOOT;

/* ============================================================
 *        (CDN. W NASTĘPNEJ CZĘŚCI)
 * ============================================================
 */
/* ============================================================
 *                  STRUKTURY DANYCH
 * ============================================================
 *  CONFIG  – parametry pracy systemu
 *  STAT    – statystyki i liczniki zdarzeń
 * ============================================================
 */

/* ------------------------------------------------------------
 *  STRUKTURA KONFIGURACJI SYSTEMU (CONFIG)
 * ------------------------------------------------------------
 *  To jest JEDYNE źródło parametrów pracy systemu
 * ------------------------------------------------------------
 */

struct ConfigRecord
{
  uint16_t magic;              // identyfikator rekordu (RECORD_MAGIC_CONFIG)
  uint8_t  version;            // wersja struktury
  uint8_t  reserved;           // wyrównanie / przyszłość

  // --- Parametry środowiskowe ---
  float target_temperature;    // zadana temperatura [°C]
  float target_humidity;       // zadana wilgotność [%]

  float hysteresis_temp;       // histereza temperatury
  float hysteresis_hum;        // histereza wilgotności

  // --- Wentylacja ---
  uint32_t fan_interval_ms;    // co ile uruchamiać wentylację
  uint32_t fan_runtime_ms;     // jak długo pracuje wentylator

  // --- Pompa / nawilżanie ---
  uint32_t pump_timeout_ms;    // maksymalny czas pracy pompy
  uint32_t refill_delay_ms;    // opóźnienie ponownego napełniania

  // --- Rezerwa ---
  uint32_t reserved32[4];      // miejsce na przyszłe opcje

  // --- Integralność ---
  uint32_t crc32;              // CRC rekordu (liczone bez tego pola)
};

/* ------------------------------------------------------------
 *  STRUKTURA STATYSTYK SYSTEMU (STAT)
 * ------------------------------------------------------------
 *  Nie wpływa na start systemu
 * ------------------------------------------------------------
 */

struct StatRecord
{
  uint16_t magic;              // RECORD_MAGIC_STAT
  uint8_t  version;            // wersja struktury
  uint8_t  fault_snapshot;     // bitmapa fault_flags w momencie zapisu

  uint32_t uptime_minutes;     // czas pracy systemu
  uint32_t reset_counter;      // liczba restartów

  // --- Liczniki zdarzeń ---
  uint32_t wdt_resets;         // ilość resetów WDT
  uint32_t flash_errors;       // błędy pamięci FLASH
  uint32_t sensor_errors;      // błędy czujników
  uint32_t pump_timeouts;      // timeouty pompy

  uint32_t reserved32[4];      // rezerwa

  uint32_t crc32;              // CRC statystyk
};

/* ============================================================
 *                  KONFIGURACJA DOMYŚLNA (DEFAULT)
 * ============================================================
 *  Używana przy:
 *  - pierwszym uruchomieniu
 *  - braku FLASH
 *  - braku GOLD
 * ============================================================
 */

const ConfigRecord DEFAULT_CONFIG =
{
  RECORD_MAGIC_CONFIG,         // magic
  CONFIG_RECORD_VERSION,       // wersja
  0,                           // reserved

  // --- Parametry środowiska ---
  24.0,                        // temperatura [°C]
  75.0,                        // wilgotność [%]

  1.0,                         // histereza temp
  3.0,                         // histereza wilg

  // --- Wentylacja ---
  30UL * 60UL * 1000UL,        // co 30 minut
  2UL  * 60UL * 1000UL,        // praca 2 minuty

  // --- Pompa ---
  90UL * 1000UL,               // timeout pompy 90s
  5UL  * 60UL * 1000UL,        // opóźnienie ponownego napełniania

  // --- Rezerwa ---
  {0, 0, 0, 0},

  0                            // CRC (liczone w runtime)
};

/* ============================================================
 *              GOLD REFERENCE (ZAPIS MCU)
 * ============================================================
 *  Jednorazowy zapis poprawnej konfiguracji
 * ============================================================
 */

bool gold_reference_valid = false;    // flaga ważności GOLD
ConfigRecord gold_reference;          // rekord GOLD w MCU

/* ============================================================
 *        (CDN. W NASTĘPNEJ CZĘŚCI)
 * ============================================================
 *//* ============================================================
 *              ZMIENNE GLOBALNE SYSTEMU
 * ============================================================
 *  Centralne punkty stanu – nie dublować!
 * ============================================================
 */

// --- Aktywna konfiguracja (JEDYNE źródło parametrów) ---
ConfigRecord active_config;

// --- Źródło konfiguracji (diagnostyka / UI) ---
enum ConfigSource
{
  CONFIG_SRC_DEFAULT = 0,   // z DEFAULT_CONFIG
  CONFIG_SRC_GOLD,          // z GOLD reference (MCU)
  CONFIG_SRC_FLASH          // z pamięci FLASH
};

ConfigSource config_source = CONFIG_SRC_DEFAULT;

// --- Stan pamięci FLASH ---
bool     flash_present   = false;   // czy kość odpowiada
uint32_t flash_jedec_id  = 0;       // JEDEC ID (diagnostyka)

// --- Flagi awarii (bitmapa) ---
uint8_t fault_flags = 0;            // 0 = brak awarii

// --- Stany alarmu ---
bool alarm_active        = false;   // czy alarm aktualnie aktywny
bool alarm_acknowledged  = false;   // czy alarm został potwierdzony

// --- Stan ekranu ---
bool screen_awake        = true;    // ekran aktywny / uśpiony

/* ============================================================
 *              TIMERY OPARTE O millis()
 * ============================================================
 *  Brak delay() – tylko zegary systemowe
 * ============================================================
 */

// Ostatnia interakcja użytkownika (enkoder / przycisk)
unsigned long last_user_activity_ms = 0;

// Moment wybudzenia ekranu (blokada ACK alarmu)
unsigned long screen_wakeup_ms = 0;

// Moment wystąpienia alarmu
unsigned long alarm_start_ms = 0;

// Ostatni zapis STAT (ochrona FLASH)
unsigned long last_stat_save_ms = 0;

// Uptime (minuty) – liczone programowo
unsigned long uptime_last_tick_ms = 0;
uint32_t      uptime_minutes      = 0;

/* ============================================================
 *              ZMIENNE ROBOCZE STEROWANIA
 * ============================================================
 */

// --- Wentylacja ---
unsigned long fan_last_start_ms = 0;
bool          fan_running       = false;

// --- Pompa ---
unsigned long pump_start_ms     = 0;
bool          pump_running      = false;

// --- Nawilżacze ---
bool humidifier_running         = false;

/* ============================================================
 *        (CDN. W NASTĘPNEJ CZĘŚCI)
 * ============================================================
 */
/* ============================================================
 *          FUNKCJE NISKIEGO POZIOMU – CRC32
 * ============================================================
 *  Używane do weryfikacji CONFIG i STAT
 * ============================================================
 */

// Obliczanie CRC32 (polinom standardowy)
uint32_t crc32_update(uint32_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xEDB88320UL;
    else
      crc >>= 1;
  }
  return crc;
}

// Liczenie CRC32 dla bufora danych
uint32_t calculate_crc32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t i = 0; i < length; i++)
  {
    crc = crc32_update(crc, data[i]);
  }

  return ~crc;
}

// Liczenie CRC dla rekordu CONFIG (z pominięciem pola crc32)
uint32_t calculate_config_crc(const ConfigRecord &cfg)
{
  return calculate_crc32(
    (const uint8_t *)&cfg,
    sizeof(ConfigRecord) - sizeof(uint32_t)
  );
}

// Liczenie CRC dla rekordu STAT (z pominięciem pola crc32)
uint32_t calculate_stat_crc(const StatRecord &stat)
{
  return calculate_crc32(
    (const uint8_t *)&stat,
    sizeof(StatRecord) - sizeof(uint32_t)
  );
}

/* ============================================================
 *          FUNKCJE NISKIEGO POZIOMU – SPI FLASH
 * ============================================================
 *  TYLKO operacje bezpieczne (READ)
 * ============================================================
 */

// Odczyt JEDEC ID pamięci FLASH (W25Qxx)
uint32_t readFlashJedecID()
{
  uint32_t jedec = 0;

  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x9F);  // JEDEC ID

  uint8_t manufacturer = SPI.transfer(0x00);
  uint8_t memory_type  = SPI.transfer(0x00);
  uint8_t capacity     = SPI.transfer(0x00);

  digitalWrite(PIN_FLASH_CS, HIGH);

  jedec = ((uint32_t)manufacturer << 16) |
          ((uint32_t)memory_type  << 8)  |
           (uint32_t)capacity;

  return jedec;
}

// Odczyt rekordu CONFIG z FLASH (bez zapisu)
bool readConfigRecordFromFlash(uint32_t address, ConfigRecord &record)
{
  uint8_t *ptr = (uint8_t *)&record;

  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x03); // READ DATA
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8)  & 0xFF);
  SPI.transfer(address & 0xFF);

  for (uint16_t i = 0; i < sizeof(ConfigRecord); i++)
  {
    ptr[i] = SPI.transfer(0x00);
  }

  digitalWrite(PIN_FLASH_CS, HIGH);

  // Sprawdzenie MAGIC
  if (record.magic != RECORD_MAGIC_CONFIG)
    return false;

  // Sprawdzenie wersji
  if (record.version != CONFIG_RECORD_VERSION)
    return false;

  // Sprawdzenie CRC
  if (record.crc32 != calculate_config_crc(record))
    return false;

  return true;
}

/* ============================================================
 *        (CDN. W NASTĘPNEJ CZĘŚCI)
 * ============================================================
 */////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* ============================================================
 *                      SETUP
 * ============================================================
 *  Inicjalizacja sprzętu
 *  Brak logiki sterowania
 * ============================================================
 */

void setup()
{

  checkResetCause();   // sprawdzenie resetu
  initWatchdog();      // uruchomienie WDT


  // --- Port szeregowy (diagnostyka) ---
  Serial.begin(115200);


  // --- Inicjalizacja czujnika DHT ---
  dht.begin();


  // --- Piny przekaźników ---
  pinMode(PIN_FAN_RELAY,  OUTPUT);
  pinMode(PIN_HUM_RELAY,  OUTPUT);
  pinMode(PIN_PUMP_RELAY, OUTPUT);

  // --- Enkoder / przycisk ---
  pinMode(PIN_ENC_A,   INPUT_PULLUP);
  pinMode(PIN_ENC_B,   INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);

    // --- Czujniki poziomu wody (ACTIVE LOW) ---
  pinMode(PIN_WATER_LOW_MAIN, INPUT_PULLUP);
  pinMode(PIN_WATER_LOW_HUM,  INPUT_PULLUP);
  pinMode(PIN_WATER_HIGH_HUM, INPUT_PULLUP);


  // --- Dioda alarmowa ---
  pinMode(PIN_LED_ERROR, OUTPUT);

  // --- SPI FLASH ---
  pinMode(PIN_FLASH_CS, OUTPUT);
  digitalWrite(PIN_FLASH_CS, HIGH);

  // --- Bezpieczne stany wyjść (ACTIVE LOW) ---
  digitalWrite(PIN_FAN_RELAY,  HIGH);
  digitalWrite(PIN_HUM_RELAY,  HIGH);
  digitalWrite(PIN_PUMP_RELAY, HIGH);
  digitalWrite(PIN_LED_ERROR, LOW);

  // --- Timery ---
  unsigned long now = millis();
  last_user_activity_ms = now;
  screen_wakeup_ms      = now;
  alarm_start_ms        = 0;
  last_stat_save_ms     = 0;
  uptime_last_tick_ms   = now;

  // --- Stan początkowy ---
  system_state = STATE_BOOT;
    // --- OLED ---
  initDisplay();

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* ============================================================
 *                      LOOP
 * ============================================================
 *  Maszyna stanów systemu
 * ============================================================
 */

void loop()
{
  switch (system_state)
  {
    /* --------------------------------------------------------
     *                      BOOT
     * --------------------------------------------------------
     */
    case STATE_BOOT:
    {
      // --- Fallback absolutny ---
      active_config = DEFAULT_CONFIG;
      config_source = CONFIG_SRC_DEFAULT;

      // --- SPI + FLASH ---
      SPI.begin();
      flash_jedec_id = readFlashJedecID();

      if (flash_jedec_id != 0x000000)
      {
        flash_present = true;

        // --- Próba odczytu CONFIG z FLASH ---
        ConfigRecord flash_cfg;

        if (readConfigRecordFromFlash(FLASH_CONFIG_START, flash_cfg))
        {
          active_config = flash_cfg;
          config_source = CONFIG_SRC_FLASH;
        }
        else if (gold_reference_valid)
        {
          // --- Fallback GOLD ---
          active_config = gold_reference;
          config_source = CONFIG_SRC_GOLD;
        }
      }
      else
      {
        flash_present = false;
        fault_flags |= FAULT_SPI_FLASH;
      }

      // --- Przejście do normalnej pracy ---
      system_state = STATE_RUN;
      break;
    }

    /* --------------------------------------------------------
     *                      RUN
     * --------------------------------------------------------
     */
case STATE_RUN:
{
  if (!screen_awake)
  {
    wakeDisplay();
    last_user_activity_ms = millis();
  }

  updateUptime();
  readDHT();
  handleHumidifier();
  readWaterLevels();
  handlePump();
  handleVentilation();

  handleMenu();
  handleEncoderButton();

  handleAlarms();
  handleScreenSleep();

  drawMainScreen();
  drawMenuOverlay();
  handleGoldSave();   // długi zapis GOLD

  break;
}






    /* --------------------------------------------------------
     *                      ALARM
     * --------------------------------------------------------
     */
case STATE_ALARM:
{
  processAlarmState();   // obsługa alarmu
  break;
}


    /* --------------------------------------------------------
     *                      SLEEP
     * --------------------------------------------------------
     */
    case STATE_SLEEP:
    {
      // System pracuje, ekran wygaszony
      // Logika dojdzie później
      break;
    }

    default:
    {
      system_state = STATE_BOOT;
      break;
    }
  }
}

/* ============================================================
 *        (CDN. W NASTĘPNEJ CZĘŚCI)
 * ============================================================
 */

/* ============================================================
 *              LOGIKA RUN – CZAS I WENTYLACJA
 * ============================================================
 *  - uptime liczone programowo
 *  - wentylacja cykliczna (niezależna od temperatury)
 * ============================================================
 */

// Aktualizacja czasu pracy systemu (minuty)
void updateUptime()
{
  unsigned long now = millis();

  // Co 60 sekund zwiększamy licznik minut
  if (now - uptime_last_tick_ms >= 60000UL)
  {
    uptime_last_tick_ms += 60000UL;
    uptime_minutes++;
  }
}

// Obsługa wentylacji cyklicznej
void handleVentilation()
{
  unsigned long now = millis();

  // Jeśli wentylator nie pracuje
  if (!fan_running)
  {
    // Czy nadszedł czas uruchomienia?
    if (now - fan_last_start_ms >= active_config.fan_interval_ms)
    {
      digitalWrite(PIN_FAN_RELAY, LOW);   // START wentylatora
      fan_running       = true;
      fan_last_start_ms = now;
    }
  }
  else
  {
    // Wentylator pracuje – sprawdzamy czas pracy
    if (now - fan_last_start_ms >= active_config.fan_runtime_ms)
    {
      digitalWrite(PIN_FAN_RELAY, HIGH);  // STOP wentylatora
      fan_running       = false;
      fan_last_start_ms = now;            // reset licznika do kolejnego cyklu
    }
  }
}
/* ============================================================
 *          LOGIKA RUN – WILGOTNOŚĆ I NAWILŻACZE
 * ============================================================
 *  - sterowanie na podstawie wilgotności
 *  - histereza zapobiegająca "klapaniu"
 * ============================================================
 */

// --- Zmienne pomiarowe ---
float current_humidity    = 0.0f;
float current_temperature = 0.0f;
bool  dht_error           = false;

// Odczyt czujnika DHT (z zabezpieczeniem)
void readDHT()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    dht_error = true;
    fault_flags |= FAULT_SENSOR_ERROR;
    return;
  }

  dht_error = false;
  current_humidity    = h;
  current_temperature = t;
}

// Obsługa nawilżaczy (histereza)
void handleHumidifier()
{
  if (dht_error)
    return; // brak sterowania przy błędzie czujnika

  // Dolny próg załączenia
  float hum_on  = active_config.target_humidity - active_config.hysteresis_hum;
  // Górny próg wyłączenia
  float hum_off = active_config.target_humidity + active_config.hysteresis_hum;

  // Jeśli nawilżacz nie pracuje
  if (!humidifier_running)
  {
    if (current_humidity < hum_on)
    {
      digitalWrite(PIN_HUM_RELAY, LOW);  // START nawilżaczy
      humidifier_running = true;
    }
  }
  else
  {
    if (current_humidity > hum_off)
    {
      digitalWrite(PIN_HUM_RELAY, HIGH); // STOP nawilżaczy
      humidifier_running = false;
    }
  }
}
/* ============================================================
 *          LOGIKA RUN – POMPA I POZIOM WODY
 * ============================================================
 *  - uzupełnianie wody dla nawilżaczy
 *  - zabezpieczenie przed suchobiegiem
 *  - timeout bezpieczeństwa
 * ============================================================
 */

// --- Piny czujników poziomu ---
#define PIN_WATER_LOW_MAIN     A0   // niski poziom – zbiornik główny
#define PIN_WATER_LOW_HUM      A1   // niski poziom – nawilżacze
#define PIN_WATER_HIGH_HUM     A2   // wysoki poziom – nawilżacze

// --- Stany czujników ---
bool water_low_main  = false;
bool water_low_hum   = false;
bool water_high_hum  = false;

// Odczyt czujników poziomu (ACTIVE LOW – zwierają do masy)
void readWaterLevels()
{
  water_low_main = (digitalRead(PIN_WATER_LOW_MAIN)  == LOW);
  water_low_hum  = (digitalRead(PIN_WATER_LOW_HUM)   == LOW);
  water_high_hum = (digitalRead(PIN_WATER_HIGH_HUM)  == LOW);
}

// Obsługa pompy
void handlePump()
{
  unsigned long now = millis();

  // Jeśli pompa NIE pracuje
  if (!pump_running)
  {
    // Warunek startu:
    // - niski poziom w nawilżaczach
    // - jest woda w zbiorniku głównym
    if (water_low_hum && !water_low_main)
    {
      digitalWrite(PIN_PUMP_RELAY, LOW); // START pompy
      pump_running  = true;
      pump_start_ms = now;
    }
  }
  else
  {
    // Pompa pracuje – sprawdzamy warunki stopu

    // 1. Osiągnięto wysoki poziom
    if (water_high_hum)
    {
      digitalWrite(PIN_PUMP_RELAY, HIGH); // STOP pompy
      pump_running = false;
    }
    // 2. Brak wody w zbiorniku głównym (suchobieg)
    else if (water_low_main)
    {
      digitalWrite(PIN_PUMP_RELAY, HIGH); // STOP pompy
      pump_running = false;
      fault_flags |= FAULT_PUMP_TIMEOUT;  // traktujemy jako awarię
    }
    // 3. Timeout bezpieczeństwa
    else if (now - pump_start_ms >= active_config.pump_timeout_ms)
    {
      digitalWrite(PIN_PUMP_RELAY, HIGH); // STOP pompy
      pump_running = false;
      fault_flags |= FAULT_PUMP_TIMEOUT;
    }
  }
}
/* ============================================================
 *          LOGIKA ALARMÓW SYSTEMOWYCH
 * ============================================================
 *  - zbieranie flag awarii
 *  - dioda AWARIA
 *  - przejście do STATE_ALARM
 * ============================================================
 */

// Sprawdzenie czy występuje jakakolwiek awaria
bool isAnyFaultActive()
{
  return (fault_flags != 0);
}

// Aktualizacja diody AWARIA
void updateErrorLed()
{
  if (isAnyFaultActive())
  {
    digitalWrite(PIN_LED_ERROR, HIGH);   // dioda ON – awaria
  }
  else
  {
    digitalWrite(PIN_LED_ERROR, LOW);    // dioda OFF – OK
  }
}

// Obsługa przejścia w stan ALARM
void handleAlarms()
{
  // Jeśli pojawiła się awaria i nie jesteśmy jeszcze w ALARM
  if (isAnyFaultActive() && system_state != STATE_ALARM)
  {
    alarm_active       = true;           // alarm aktywny
    alarm_acknowledged = false;          // niepotwierdzony
    alarm_start_ms     = millis();       // czas wystąpienia alarmu

    system_state = STATE_ALARM;          // przejście do ALARM
  }
}

// Obsługa stanu ALARM (bez UI – jeszcze)
void processAlarmState()
{
  // Aktualizacja diody awarii
  updateErrorLed();

  // Jeśli wszystkie awarie zniknęły
  if (!isAnyFaultActive())
  {
    alarm_active       = false;
    alarm_acknowledged = false;

    system_state = STATE_RUN;            // powrót do RUN
  }
}
/* ============================================================
 *          OLED – WYŚWIETLACZ I OBSŁUGA EKRANU
 * ============================================================
 *  - ekran główny
 *  - sen po bezczynności
 *  - wybudzanie dowolnym ruchem
 * ============================================================
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// --- Konfiguracja OLED ---
#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_RESET   -1

Adafruit_SH1106G display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// Inicjalizacja OLED
void initDisplay()
{
  if (!display.begin(0x3C, true))
  {
    fault_flags |= FAULT_SPI_FLASH; // używamy jako ogólna awaria HW
    return;
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.display();
}

// Wygaszanie ekranu
void sleepDisplay()
{
  display.clearDisplay();
  display.display();
  screen_awake = false;
}

// Wybudzanie ekranu
void wakeDisplay()
{
  screen_awake = true;
  screen_wakeup_ms = millis();
}

// Sprawdzenie bezczynności i sen
void handleScreenSleep()
{
  if (!screen_awake)
    return;

  unsigned long now = millis();

  if (now - last_user_activity_ms >= active_config.fan_interval_ms) // tymczasowo używamy tego czasu
  {
    sleepDisplay();
  }
}

// Rysowanie ekranu głównego
void drawMainScreen()
{
  if (!screen_awake)
    return;

  display.clearDisplay();

  // --- Linia 1: temperatura i wilgotność ---
  display.setCursor(0, 0);
  display.print("T:");
  display.print(current_temperature, 1);
  display.print("C ");

  display.print("H:");
  display.print(current_humidity, 0);
  display.print("%");

  // --- Linia 2: nawilzacz / wentylator ---
  display.setCursor(0, 16);
  display.print("HUM:");
  display.print(humidifier_running ? "ON " : "OFF");

  display.print(" FAN:");
  display.print(fan_running ? "ON" : "OFF");

  // --- Linia 3: pompa ---
  display.setCursor(0, 32);
  display.print("PUMP:");
  display.print(pump_running ? "ON" : "OFF");

  // --- Linia 4: alarm / zrodlo config ---
  display.setCursor(0, 48);

  if (isAnyFaultActive())
    display.print("! ALARM ");
  else
    display.print("OK ");

  switch (config_source)
  {
    case CONFIG_SRC_FLASH:   display.print("FLASH");   break;
    case CONFIG_SRC_GOLD:    display.print("GOLD");    break;
    default:                 display.print("DEFAULT"); break;
  }

  display.display();
}
/* ============================================================
 *          ENKODER + MENU UŻYTKOWNIKA
 * ============================================================
 *  - zmiana parametrów bez zapisu
 *  - zapis będzie w kolejnej części
 * ============================================================
 */

// --- Parametry enkodera ---
#define ENC_STEP_TEMP      0.5f
#define ENC_STEP_HUM       1.0f

// --- Menu ---
enum MenuPage
{
  MENU_MAIN = 0,
  MENU_TEMP,
  MENU_HUM,
  MENU_FAN
};

MenuPage menu_page = MENU_MAIN;

// --- Enkoder ---
int last_enc_a = HIGH;
unsigned long last_enc_event_ms = 0;

// Odczyt enkodera (prosty, stabilny)
int readEncoderDelta()
{
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);

  int delta = 0;

  if (a != last_enc_a)
  {
    if (b != a)
      delta = 1;
    else
      delta = -1;

    last_enc_a = a;
    last_enc_event_ms = millis();
    last_user_activity_ms = last_enc_event_ms;
  }

  return delta;
}

// Obsługa menu i zmian parametrów
void handleMenu()
{
  int delta = readEncoderDelta();
  if (delta == 0)
    return;

  switch (menu_page)
  {
    case MENU_MAIN:
      menu_page = MENU_TEMP;
      break;

    case MENU_TEMP:
      active_config.target_temperature += delta * ENC_STEP_TEMP;
      break;

    case MENU_HUM:
      active_config.target_humidity += delta * ENC_STEP_HUM;
      break;

    case MENU_FAN:
      active_config.fan_interval_ms += delta * 5UL * 60000UL;
      break;
  }
    saveConfigToFlash();   // zapis po każdej zmianie

}

// Przycisk enkodera – zmiana strony menu
void handleEncoderButton()
{
  static bool last_btn = HIGH;
  bool btn = digitalRead(PIN_ENC_BTN);

  if (last_btn == HIGH && btn == LOW)
  {
    last_user_activity_ms = millis();

    menu_page = (MenuPage)((menu_page + 1) % 4);
  }

  last_btn = btn;
}

// Rysowanie menu (nakładka na ekran główny)
void drawMenuOverlay()
{
  if (!screen_awake)
    return;

  display.setCursor(90, 0);

  switch (menu_page)
  {
    case MENU_MAIN: display.print("MAIN"); break;
    case MENU_TEMP: display.print("TEMP"); break;
    case MENU_HUM:  display.print("HUM "); break;
    case MENU_FAN:  display.print("FAN "); break;
  }
}
/* ============================================================
 *          ZAPIS KONFIGURACJI – FLASH + GOLD
 * ============================================================
 *  - rotacja rekordów CONFIG w FLASH
 *  - zapis GOLD tylko po długim przytrzymaniu
 * ============================================================
 */

// --- Parametry zapisu ---
#define CONFIG_ROTATION_SLOTS   (FLASH_CONFIG_PAGES)   // 1 rekord / strona
#define GOLD_HOLD_TIME_MS       8000UL                 // 8 sekund

// --- Stan zapisu ---
uint32_t config_sequence = 0;      // licznik sekwencji CONFIG
uint32_t last_config_addr = 0;     // adres ostatniego zapisu

// --- Stan przycisku do GOLD ---
unsigned long enc_btn_hold_ms = 0;
bool enc_btn_holding = false;

/* ------------------------------------------------------------
 *  Niskopoziomowy zapis strony FLASH (W25Qxx)
 * ------------------------------------------------------------
 */

void flashWriteEnable()
{
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x06); // WRITE ENABLE
  digitalWrite(PIN_FLASH_CS, HIGH);
}

void flashWaitBusy()
{
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x05); // READ STATUS
  while (SPI.transfer(0x00) & 0x01) {}
  digitalWrite(PIN_FLASH_CS, HIGH);
}

void flashEraseSector(uint32_t address)
{
  flashWriteEnable();

  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x20); // SECTOR ERASE 4kB
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8)  & 0xFF);
  SPI.transfer(address & 0xFF);
  digitalWrite(PIN_FLASH_CS, HIGH);

  flashWaitBusy();
}

void flashWriteData(uint32_t address, const uint8_t *data, uint16_t len)
{
  flashWriteEnable();

  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x02); // PAGE PROGRAM
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8)  & 0xFF);
  SPI.transfer(address & 0xFF);

  for (uint16_t i = 0; i < len; i++)
    SPI.transfer(data[i]);

  digitalWrite(PIN_FLASH_CS, HIGH);
  flashWaitBusy();
}

/* ------------------------------------------------------------
 *  Zapis CONFIG do FLASH (rotacja)
 * ------------------------------------------------------------
 */

void saveConfigToFlash()
{
  if (!flash_present)
    return;

  // Obliczenie CRC
  active_config.crc32 = calculate_config_crc(active_config);

  // Ustalenie slotu
  uint32_t slot = config_sequence % CONFIG_ROTATION_SLOTS;
  uint32_t addr = FLASH_CONFIG_START + slot * FLASH_PAGE_SIZE;

  // Kasowanie i zapis
  flashEraseSector(addr);
  flashWriteData(addr, (uint8_t *)&active_config, sizeof(ConfigRecord));

  last_config_addr = addr;
  config_sequence++;
}

/* ------------------------------------------------------------
 *  Obsługa długiego przytrzymania – zapis GOLD
 * ------------------------------------------------------------
 */

void handleGoldSave()
{
  bool btn = digitalRead(PIN_ENC_BTN);

  if (btn == LOW)
  {
    if (!enc_btn_holding)
    {
      enc_btn_holding = true;
      enc_btn_hold_ms = millis();
    }
    else if (millis() - enc_btn_hold_ms >= GOLD_HOLD_TIME_MS)
    {
      // --- ZAPIS GOLD ---
      gold_reference = active_config;
      gold_reference_valid = true;

      enc_btn_holding = false;
    }
  }
  else
  {
    enc_btn_holding = false;
  }
}
/* ============================================================
 *          WATCHDOG + FINALNE DOMKNIĘCIE SYSTEMU
 * ============================================================
 *  - reset przy zawisie
 *  - oznaczenie resetu WDT
 *  - finalny polish v1.0
 * ============================================================
 */

#include <avr/wdt.h>

/* ------------------------------------------------------------
 *  Inicjalizacja Watchdoga
 * ------------------------------------------------------------
 *  8 sekund – bezpieczne dla całej pętli RUN
 * ------------------------------------------------------------
 */

void initWatchdog()
{
  wdt_disable();                 // na wszelki wypadek
  wdt_enable(WDTO_8S);           // watchdog 8 sekund
}

/* ------------------------------------------------------------
 *  Sprawdzenie przyczyny resetu
 * ------------------------------------------------------------
 */

void checkResetCause()
{
  if (MCUSR & _BV(WDRF))
  {
    // Reset z Watchdoga
    fault_flags |= FAULT_WDT_RESET;
  }

  MCUSR = 0; // kasujemy flagi resetu
}

/* ------------------------------------------------------------
 *  Odświeżanie Watchdoga
 * ------------------------------------------------------------
 */

void kickWatchdog()
{
  wdt_reset();
}
