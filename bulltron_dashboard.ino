#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lvgl.h>
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <map>
#include <string.h>
#include <LittleFS.h>
#include "Audio.h"

// ==================================================================
// AUDIO: I2S-Pins des onboard Lautsprecher-JST-Anschlusses
// (community-verifiziert: BCLK=36, LRC=35, DOUT=37, Quad-PSRAM-Variante,
// kein Konflikt mit dem PSRAM)
// ==================================================================
#define I2S_BCLK 36
#define I2S_LRC  35
#define I2S_DOUT 37

Audio audio;
volatile bool triggerConnectSound = false; // wird von Core 0 (BLE) gesetzt, von Core 1 (loop) abgeholt


// ==================================================================
// TEIL 2: DATENMODELL - wird von Core 0 (BLE) geschrieben,
// von Core 1 (UI) gelesen
// ==================================================================

struct BmsData {
    bool isConnected;
    float cellV[4];
    int cellCount;
    float packVoltage;
    float cellMax, cellMin;
    int temp1, temp2;
    uint8_t soc;              // 0-100 (mit Nachkommastelle * 10 intern gerundet)
    float socPrecise;         // z.B. 48.9
    float current;            // A, negativ = Entladen, positiv = Laden
    float remainAh;
    int chargeAllowed;        // 0=Aus,1=An,-1=unbekannt
    int dischargeAllowed;     // 0=Aus,1=An,-1=unbekannt
    int heaterOn;             // -1=unbekannt (TODO), 0=Aus, 1=An
    int timeHours;            // Restlaufzeit ODER Zeit-bis-voll, je nach chargingDirection
    int timeMinutes;
    bool isCharging;          // true=laedt, false=entlaedt/idle
    int cycles;                // Ladezyklen
};

volatile BmsData currentBmsState = {}; // Nullinitialisierung aller Felder


// ==================================================================
// TEIL 1: DISPLAY-TREIBER (WT32-SC01 Plus, ESP32-S3, ST7796, FT5x06)
// Pin-Belegung community-verifiziert (sukesh-ak/ESP32-TUX)
// ==================================================================

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ST7796  _panel_instance;
    lgfx::Bus_Parallel8 _bus_instance;
    lgfx::Light_PWM     _light_instance;
    lgfx::Touch_FT5x06  _touch_instance;

public:
    LGFX(void)
    {
        {
            auto cfg = _bus_instance.config();
            cfg.freq_write = 40000000;
            cfg.pin_wr = 47;
            cfg.pin_rd = -1;
            cfg.pin_rs = 0;
            cfg.pin_d0 = 9;
            cfg.pin_d1 = 46;
            cfg.pin_d2 = 3;
            cfg.pin_d3 = 8;
            cfg.pin_d4 = 18;
            cfg.pin_d5 = 17;
            cfg.pin_d6 = 16;
            cfg.pin_d7 = 15;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs = -1;
            cfg.pin_rst = 4;
            cfg.pin_busy = -1;
            cfg.panel_width = 320;
            cfg.panel_height = 480;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.readable = false;
            cfg.invert = true;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = false;
            _panel_instance.config(cfg);
        }
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl = 45;
            cfg.invert = false;
            cfg.freq = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }
        {
            auto cfg = _touch_instance.config();
            cfg.x_min = 0;
            cfg.x_max = 319;
            cfg.y_min = 0;
            cfg.y_max = 479;
            cfg.pin_int = 7;
            cfg.bus_shared = true;
            cfg.offset_rotation = 0;
            cfg.i2c_port = 1;
            cfg.i2c_addr = 0x38;
            cfg.pin_sda = 6;
            cfg.pin_scl = 5;
            cfg.freq = 400000;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }
        setPanel(&_panel_instance);
    }
};

LGFX display;

static const uint16_t SCREEN_W = 320; // Hochkant, wie im nativen Panel-Format
static const uint16_t SCREEN_H = 480;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_W * 40];

void lvgl_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    display.startWrite();
    display.setAddrWindow(area->x1, area->y1, w, h);
    display.pushColors((uint16_t*)&color_p->full, w * h, true);
    display.endWrite();
    lv_disp_flush_ready(disp);
}

// --- Bildschirm-Timeout / Auto-Dimmen ---
uint32_t lastTouchMs = 0;
bool backlightOn = true;
static const uint32_t SCREEN_TIMEOUT_MS = 30000; // 30s ohne Beruehrung -> abdunkeln
static const uint8_t BRIGHTNESS_NORMAL = 255;
static const uint8_t BRIGHTNESS_DIMMED = 20;      // 0 = komplett aus, z.B. 20 fuer nur Abdunkeln

void lvgl_touch_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
    int32_t x, y;
    if (display.getTouch(&x, &y)) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;

        lastTouchMs = millis();
        if (!backlightOn) {
            display.setBrightness(BRIGHTNESS_NORMAL);
            backlightOn = true;
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}


// ==================================================================
// TEIL 3: BLE BACKEND (Core 0) - Daly D2 Protokoll ueber BullTron BMS
// ==================================================================

static NimBLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
static NimBLEUUID notifyUUID ("0000fff1-0000-1000-8000-00805f9b34fb");
static NimBLEUUID writeUUID  ("0000fff2-0000-1000-8000-00805f9b34fb");

static const uint8_t HANDSHAKE_CMD[]  = {0xD2, 0x10, 0x00, 0xD4, 0x00, 0x03, 0x1A, 0x08, 0x1A, 0x14, 0x0F, 0x17, 0x76, 0xA7};
static const uint8_t STATUS_CMD[]     = {0xD2, 0x03, 0x00, 0x00, 0x00, 0x3E, 0xD7, 0xB9};
static const uint8_t EXT_STATUS_CMD[] = {0xD2, 0x03, 0x00, 0x80, 0x00, 0x29, 0x96, 0x5F};
static const float TOTAL_CAPACITY_AH = 105.0f;

NimBLEAddress* targetAddress = nullptr;
std::string targetName = "";
bool deviceFound = false;
volatile bool handshakeAckReceived = false;
NimBLERemoteCharacteristic* g_writeChar = nullptr;

class ScanCallbacks: public NimBLEScanCallbacks {
    void onDiscovered(const NimBLEAdvertisedDevice* dev) override { handleDevice(dev); }
    void onResult(const NimBLEAdvertisedDevice* dev) override { handleDevice(dev); }

    void handleDevice(const NimBLEAdvertisedDevice* dev) {
        std::string name = dev->getName();
        std::string addr = dev->getAddress().toString();
        bool hasName = !name.empty();
        static std::map<std::string, bool> seen;
        auto it = seen.find(addr);
        if (it != seen.end() && (it->second || !hasName)) return;
        seen[addr] = hasName;

        std::string lower = name;
        for (auto &c : lower) c = (char)tolower((unsigned char)c);
        if (!deviceFound && lower.find("bulltron") != std::string::npos) {
            targetAddress = new NimBLEAddress(dev->getAddress());
            targetName = name;
            deviceFound = true;
        }
    }
};

bool waitForFlag(volatile bool& flag, uint32_t timeoutMs) {
    uint32_t start = millis();
    while (!flag && (millis() - start) < timeoutMs) delay(50);
    return flag;
}

void parseStatusFrame(const uint8_t* data, size_t length) {
    if (length < 3 || data[0] != 0xD2 || data[1] != 0x03) return;
    uint8_t byteCount = data[2];
    const uint8_t* payload = data + 3;
    if (length < (size_t)(3 + byteCount)) return;
    if (byteCount != 0x7c) return; // nur der 124-Byte-Hauptframe wird ausgewertet

    BmsData d;
    memcpy((void*)&d, (const void*)&currentBmsState, sizeof(BmsData)); // lokale Kopie als Basis
    d.isConnected = true;

    float cellSum = 0;
    int cellCount = 0;
    float cellMax = 0, cellMin = 0;
    for (int i = 0; i < 16 && (2 * i + 1) < byteCount; i++) {
        uint16_t raw = (payload[2 * i] << 8) | payload[2 * i + 1];
        if (raw == 0) continue;
        float v = raw / 1000.0f;
        if (cellCount < 4) d.cellV[cellCount] = v;
        cellSum += v;
        if (cellCount == 0 || v > cellMax) cellMax = v;
        if (cellCount == 0 || v < cellMin) cellMin = v;
        cellCount++;
    }
    d.cellCount = cellCount;
    d.packVoltage = cellSum;
    d.cellMax = cellMax;
    d.cellMin = cellMin;

    if (byteCount >= 68) d.temp1 = (int)payload[65] - 40;
    if (byteCount >= 70) d.temp2 = (int)payload[67] - 40;

    if (byteCount >= 86) {
        uint16_t socRaw = (payload[84] << 8) | payload[85];
        d.socPrecise = socRaw / 10.0f;
        d.soc = (uint8_t)(d.socPrecise + 0.5f);
    }

    if (byteCount >= 84) {
        uint16_t currentRaw = (payload[82] << 8) | payload[83];
        d.current = (currentRaw - 30000) / 10.0f;
        d.isCharging = d.current > 0.01f;
    }

    if (byteCount >= 98) {
        uint16_t remainAhRaw = (payload[96] << 8) | payload[97];
        d.remainAh = remainAhRaw / 10.0f;
    }

    if (byteCount >= 110) {
        d.chargeAllowed = payload[107] != 0 ? 1 : 0;
        d.dischargeAllowed = payload[109] != 0 ? 1 : 0;
    }

    if (byteCount >= 104) d.cycles = payload[103];

    // Restzeit
    d.timeHours = 0; d.timeMinutes = 0;
    if (d.current < -0.01f) {
        float h = d.remainAh / fabs(d.current);
        d.timeHours = (int)h;
        d.timeMinutes = (int)((h - d.timeHours) * 60);
    } else if (d.current > 0.01f) {
        float ahToFull = TOTAL_CAPACITY_AH - d.remainAh;
        if (ahToFull < 0) ahToFull = 0;
        float h = ahToFull / d.current;
        d.timeHours = (int)h;
        d.timeMinutes = (int)((h - d.timeHours) * 60);
    }

    // d.heaterOn bleibt -1 (unbekannt) - TODO sobald im Winter identifiziert

    memcpy((void*)&currentBmsState, (const void*)&d, sizeof(BmsData));
}

void notifyCallback(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t length, bool isNotify) {
    if (length >= 6 && data[0] == 0xD2 && data[1] == 0x10 && data[2] == 0x00 && data[3] == 0xD4) {
        handshakeAckReceived = true;
        return;
    }
    parseStatusFrame(data, length);
}

void bleWorkerTask(void* parameter) {
    NimBLEDevice::init("ESP32-BullTron");

    while (true) {
        // --- Scan-Phase ---
        deviceFound = false;
        NimBLEScan* scan = NimBLEDevice::getScan();
        scan->setScanCallbacks(new ScanCallbacks(), false);
        scan->setInterval(100);
        scan->setWindow(99);
        scan->setActiveScan(true);
        scan->setMaxResults(0);
        scan->start(12000, false);
        delay(13000);

        if (!deviceFound) {
            currentBmsState.isConnected = false;
            delay(3000);
            continue;
        }

        // --- Verbindungs-Phase ---
        NimBLEClient* client = NimBLEDevice::createClient();
        if (!client->connect(*targetAddress)) {
            currentBmsState.isConnected = false;
            NimBLEDevice::deleteClient(client);
            delay(5000);
            continue;
        }

        NimBLERemoteService* svc = client->getService(serviceUUID);
        if (!svc) { client->disconnect(); delay(5000); continue; }
        NimBLERemoteCharacteristic* notifyChar = svc->getCharacteristic(notifyUUID);
        g_writeChar = svc->getCharacteristic(writeUUID);
        if (!notifyChar || !g_writeChar) { client->disconnect(); delay(5000); continue; }

        notifyChar->subscribe(true, notifyCallback);
        delay(500);

        handshakeAckReceived = false;
        g_writeChar->writeValue(HANDSHAKE_CMD, sizeof(HANDSHAKE_CMD), false);
        if (!waitForFlag(handshakeAckReceived, 3000)) {
            client->disconnect();
            delay(3000);
            continue;
        }

        // Verbindung erfolgreich aufgebaut - Sound-Trigger setzen. Wichtig:
        // audio.* wird bewusst NICHT hier (Core 0) aufgerufen, sondern nur
        // dieses Flag gesetzt - die eigentliche Wiedergabe passiert in loop()
        // auf Core 1, damit alle Audio-Bibliotheksaufrufe auf demselben Kern
        // bleiben (die Bibliothek ist nicht fuer Cross-Core-Zugriff ausgelegt).
        triggerConnectSound = true;

        // --- Polling-Phase: alle 5s beide Kommandos senden (wie Original-App) ---
        while (client->isConnected()) {
            g_writeChar->writeValue(EXT_STATUS_CMD, sizeof(EXT_STATUS_CMD), false);
            delay(500);
            g_writeChar->writeValue(STATUS_CMD, sizeof(STATUS_CMD), false);
            delay(2000);
            delay(2500);
        }

        currentBmsState.isConnected = false;
        NimBLEDevice::deleteClient(client);
        delay(2000);
    }
}

// ==================================================================
// TEIL 4: UI (Core 1) - Ladezustand-Ring wie in der BullTron-App
// ==================================================================

lv_obj_t* header_bar;
lv_obj_t* scr_main;
lv_obj_t* scr_history;
lv_obj_t* chart_history;
lv_chart_series_t* series_soc;
lv_chart_series_t* series_current;

// Verlaufs-Diagramm: 1 Messpunkt alle SAMPLE_INTERVAL_MS, LVGL verwaltet den
// Ringpuffer intern (lv_chart_set_point_count) - kein eigenes Array noetig.
static const int HISTORY_SIZE = 120;
static const uint32_t SAMPLE_INTERVAL_MS = 60000; // 1x/Minute -> 2h Verlauf bei 120 Punkten
uint32_t lastSampleMs = 0;

lv_obj_t* arc_seg_red;
lv_obj_t* arc_seg_orange;
lv_obj_t* arc_seg_yellow;
lv_obj_t* arc_seg_green;
lv_obj_t* arc_mask;
lv_obj_t* label_soc_pct;
lv_obj_t* label_ah;
lv_obj_t* icon_center;
lv_obj_t* label_status;
lv_obj_t* label_temp;
lv_obj_t* label_power;
lv_obj_t* label_voltage;
lv_obj_t* label_current;
lv_obj_t* label_time;
lv_obj_t* label_charge_val;
lv_obj_t* label_discharge_val;
lv_obj_t* label_heater_val;
lv_obj_t* label_cycles_val;
lv_obj_t* label_maxv_val;
lv_obj_t* label_minv_val;
lv_obj_t* label_diffv_val;
lv_obj_t* label_cellcount;
lv_obj_t* cell_bars[4];
lv_obj_t* cell_labels[4];

// BullTron-Gruen, aus dem Screenshot abgeschaetzt
#define COL_BRAND      lv_color_hex(0x8DC63F)
#define COL_BG         lv_color_hex(0x14161A)
#define COL_PANEL      lv_color_hex(0x1E2126)
#define COL_TEXT       lv_color_hex(0xE8E8E8)
#define COL_TEXT_DIM   lv_color_hex(0x8A8D93)
#define COL_ARC_GRAY   lv_color_hex(0x2A2E35)
#define COL_RED        lv_color_hex(0xE74C3C)
#define COL_ORANGE     lv_color_hex(0xE67E22)
#define COL_YELLOW     lv_color_hex(0xF1C40F)
#define COL_GREEN      lv_color_hex(0x2ECC71)

// Erzeugt einen der vier Farb-Segmentboegen (fester Regenbogen-Hintergrund,
// wie im BullTron-Ring). Nur der MAIN-Teil (Hintergrundbogen) wird genutzt,
// der INDICATOR-Teil bleibt unsichtbar - so bekommen wir einen festen,
// mehrfarbigen Ring statt eines einzelnen Fuellstands.
lv_obj_t* create_arc_segment(int startAngle, int endAngle, lv_color_t color) {
    lv_obj_t* a = lv_arc_create(lv_scr_act());
    lv_obj_set_size(a, 200, 200);
    lv_arc_set_rotation(a, 135);
    lv_arc_set_bg_angles(a, startAngle, endAngle);
    lv_arc_set_value(a, 0);
    lv_obj_align(a, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_arc_width(a, 16, LV_PART_MAIN);
    lv_obj_set_style_arc_color(a, color, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(a, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_remove_style(a, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
    return a;
}

lv_obj_t* create_row_value(lv_obj_t* parent, int x, int y, int w, const char* labelTxt) {
    lv_obj_t* lbl = lv_label_create(parent);
    lv_label_set_text(lbl, labelTxt);
    lv_obj_set_style_text_color(lbl, COL_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_width(lbl, w);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP); // schneidet ab statt in Nachbarspalte zu ueberlaufen
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, x, y);
    return lbl;
}

void build_ui() {
    lv_obj_set_style_bg_color(lv_scr_act(), COL_BG, 0);

    // --- Kopfzeile (Farbe wird dynamisch je nach Status gesetzt) ---
    header_bar = lv_obj_create(lv_scr_act());
    lv_obj_set_size(header_bar, SCREEN_W, 28);
    lv_obj_align(header_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header_bar, COL_YELLOW, 0); // Start: "Suche..."
    lv_obj_set_style_border_width(header_bar, 0, 0);
    lv_obj_set_style_radius(header_bar, 0, 0);
    lv_obj_clear_flag(header_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(header_bar);
    lv_label_set_text(title, "BullTron");
    lv_obj_set_style_text_color(title, lv_color_hex(0x1A1A1A), 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 10, 0);

    label_status = lv_label_create(header_bar);
    lv_label_set_text(label_status, "Suche...");
    lv_obj_set_style_text_color(label_status, lv_color_hex(0x1A1A1A), 0);
    lv_obj_align(label_status, LV_ALIGN_RIGHT_MID, -10, 0);

    // --- Temperatur (links) / Leistung (rechts) ---
    label_temp = lv_label_create(lv_scr_act());
    lv_label_set_text(label_temp, "--.-C");
    lv_obj_set_style_text_color(label_temp, COL_TEXT, 0);
    lv_obj_align(label_temp, LV_ALIGN_TOP_LEFT, 15, 34);

    label_power = lv_label_create(lv_scr_act());
    lv_label_set_text(label_power, "-- W");
    lv_obj_set_style_text_color(label_power, COL_TEXT, 0);
    lv_obj_align(label_power, LV_ALIGN_TOP_RIGHT, -15, 34);

    // --- Regenbogen-Ring: 4 feste Farbsegmente + grauer Masken-Bogen obendrauf ---
    arc_seg_red    = create_arc_segment(0,   67,  COL_RED);
    arc_seg_orange = create_arc_segment(67,  135, COL_ORANGE);
    arc_seg_yellow = create_arc_segment(135, 202, COL_YELLOW);
    arc_seg_green  = create_arc_segment(202, 270, COL_GREEN);
    arc_mask = create_arc_segment(270, 270, COL_ARC_GRAY); // Winkel wird live aktualisiert

    label_ah = lv_label_create(lv_scr_act());
    lv_label_set_text(label_ah, "-- Ah");
    lv_obj_set_style_text_color(label_ah, COL_TEXT, 0);
    lv_obj_align(label_ah, LV_ALIGN_TOP_MID, 0, 84);

    icon_center = lv_label_create(lv_scr_act());
    lv_label_set_text(icon_center, LV_SYMBOL_BATTERY_FULL);
    lv_obj_set_style_text_color(icon_center, COL_TEXT_DIM, 0);
    lv_obj_align(icon_center, LV_ALIGN_TOP_MID, 0, 128);

    label_soc_pct = lv_label_create(lv_scr_act());
    lv_label_set_text(label_soc_pct, "-- %");
    lv_obj_set_style_text_color(label_soc_pct, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_bg_color(label_soc_pct, COL_GREEN, 0);
    lv_obj_set_style_bg_opa(label_soc_pct, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(label_soc_pct, 12, 0);
    lv_obj_set_style_pad_hor(label_soc_pct, 10, 0);
    lv_obj_set_style_pad_ver(label_soc_pct, 3, 0);
    lv_obj_align(label_soc_pct, LV_ALIGN_TOP_MID, 0, 213);

    // --- Spannung / Strom ---
    label_voltage = lv_label_create(lv_scr_act());
    lv_label_set_text(label_voltage, "--.-- V");
    lv_obj_set_style_text_color(label_voltage, COL_TEXT, 0);
    lv_obj_align(label_voltage, LV_ALIGN_TOP_LEFT, 15, 238);

    label_current = lv_label_create(lv_scr_act());
    lv_label_set_text(label_current, "--.- A");
    lv_obj_set_style_text_color(label_current, COL_TEXT, 0);
    lv_obj_align(label_current, LV_ALIGN_TOP_RIGHT, -15, 238);

    // --- Zeit-Box ---
    lv_obj_t* time_box = lv_obj_create(lv_scr_act());
    lv_obj_set_size(time_box, SCREEN_W - 30, 30);
    lv_obj_align(time_box, LV_ALIGN_TOP_MID, 0, 268);
    lv_obj_set_style_bg_color(time_box, COL_PANEL, 0);
    lv_obj_set_style_border_color(time_box, COL_TEXT_DIM, 0);
    lv_obj_set_style_border_width(time_box, 1, 0);
    lv_obj_set_style_radius(time_box, 6, 0);
    lv_obj_clear_flag(time_box, LV_OBJ_FLAG_SCROLLABLE);

    label_time = lv_label_create(time_box);
    lv_label_set_text(label_time, "");
    lv_obj_set_style_text_color(label_time, COL_TEXT, 0);
    lv_obj_center(label_time);

    // --- Laden / Entladen / Heizung / Zyklen (4 Spalten) ---
    int colW = (SCREEN_W - 20) / 4;
    lv_obj_t* row2 = lv_obj_create(lv_scr_act());
    lv_obj_set_size(row2, SCREEN_W - 20, 40);
    lv_obj_align(row2, LV_ALIGN_TOP_MID, 0, 304);
    lv_obj_set_style_bg_color(row2, COL_PANEL, 0);
    lv_obj_set_style_border_width(row2, 0, 0);
    lv_obj_set_style_radius(row2, 6, 0);
    lv_obj_clear_flag(row2, LV_OBJ_FLAG_SCROLLABLE);

    create_row_value(row2, 8,               -12, colW - 10, "Laden");
    create_row_value(row2, 8 + colW,        -12, colW - 10, "Entladen");
    create_row_value(row2, 8 + colW * 2,    -12, colW - 10, "Heizung");
    create_row_value(row2, 8 + colW * 3,    -12, colW - 10, "Zyklen");

    label_charge_val = lv_label_create(row2);
    lv_obj_set_style_text_font(label_charge_val, &lv_font_montserrat_10, 0);
    lv_label_set_text(label_charge_val, "--");
    lv_obj_set_width(label_charge_val, colW - 10);
    lv_label_set_long_mode(label_charge_val, LV_LABEL_LONG_CLIP);
    lv_obj_align(label_charge_val, LV_ALIGN_TOP_LEFT, 8, 12);

    label_discharge_val = lv_label_create(row2);
    lv_label_set_text(label_discharge_val, "--");
    lv_obj_set_width(label_discharge_val, colW - 10);
    lv_label_set_long_mode(label_discharge_val, LV_LABEL_LONG_CLIP);
    lv_obj_align(label_discharge_val, LV_ALIGN_TOP_LEFT, 8 + colW, 12);

    label_heater_val = lv_label_create(row2);
    lv_obj_set_style_text_font(label_heater_val, &lv_font_montserrat_10, 0);
    lv_label_set_text(label_heater_val, "unbek.");
    lv_obj_set_style_text_color(label_heater_val, COL_TEXT_DIM, 0);
    lv_obj_set_width(label_heater_val, colW - 10);
    lv_label_set_long_mode(label_heater_val, LV_LABEL_LONG_CLIP);
    lv_obj_align(label_heater_val, LV_ALIGN_TOP_LEFT, 8 + colW * 2, 12);

    label_cycles_val = lv_label_create(row2);
    lv_label_set_text(label_cycles_val, "--");
    lv_obj_set_style_text_color(label_cycles_val, COL_TEXT, 0);
    lv_obj_set_width(label_cycles_val, colW - 10);
    lv_label_set_long_mode(label_cycles_val, LV_LABEL_LONG_CLIP);
    lv_obj_align(label_cycles_val, LV_ALIGN_TOP_LEFT, 8 + colW * 3, 12);

    // --- Max/Min-Spannung / Differenz (3 Spalten) ---
    int colW3 = (SCREEN_W - 20) / 3;
    lv_obj_t* row3 = lv_obj_create(lv_scr_act());
    lv_obj_set_size(row3, SCREEN_W - 20, 40);
    lv_obj_align(row3, LV_ALIGN_TOP_MID, 0, 356);
    lv_obj_set_style_bg_color(row3, COL_PANEL, 0);
    lv_obj_set_style_border_width(row3, 0, 0);
    lv_obj_set_style_radius(row3, 6, 0);
    lv_obj_clear_flag(row3, LV_OBJ_FLAG_SCROLLABLE);

    create_row_value(row3, 8,             -12, colW3 - 10, "Max-V");
    create_row_value(row3, 8 + colW3,     -12, colW3 - 10, "Min-V");
    create_row_value(row3, 8 + colW3 * 2, -12, colW3 - 10, "Diff-V");

    label_maxv_val = lv_label_create(row3);
    lv_label_set_text(label_maxv_val, "--.--- V");
    lv_obj_set_style_text_color(label_maxv_val, COL_TEXT, 0);
    lv_obj_set_width(label_maxv_val, colW3 - 10);
    lv_label_set_long_mode(label_maxv_val, LV_LABEL_LONG_CLIP);
    lv_obj_align(label_maxv_val, LV_ALIGN_TOP_LEFT, 8, 12);

    label_minv_val = lv_label_create(row3);
    lv_label_set_text(label_minv_val, "--.--- V");
    lv_obj_set_style_text_color(label_minv_val, COL_TEXT, 0);
    lv_obj_set_width(label_minv_val, colW3 - 10);
    lv_label_set_long_mode(label_minv_val, LV_LABEL_LONG_CLIP);
    lv_obj_align(label_minv_val, LV_ALIGN_TOP_LEFT, 8 + colW3, 12);

    label_diffv_val = lv_label_create(row3);
    lv_label_set_text(label_diffv_val, "--.--- V");
    lv_obj_set_style_text_color(label_diffv_val, COL_TEXT, 0);
    lv_obj_set_width(label_diffv_val, colW3 - 10);
    lv_label_set_long_mode(label_diffv_val, LV_LABEL_LONG_CLIP);
    lv_obj_align(label_diffv_val, LV_ALIGN_TOP_LEFT, 8 + colW3 * 2, 12);

    // --- Anzahl Zellen + Einzelspannungen ---
    label_cellcount = lv_label_create(lv_scr_act());
    lv_label_set_text(label_cellcount, "Anzahl Zellen: --");
    lv_obj_set_style_text_color(label_cellcount, COL_TEXT_DIM, 0);
    lv_obj_align(label_cellcount, LV_ALIGN_TOP_LEFT, 15, 408);

    int cellW = (SCREEN_W - 30) / 4;
    for (int i = 0; i < 4; i++) {
        cell_bars[i] = lv_bar_create(lv_scr_act());
        lv_obj_set_size(cell_bars[i], cellW - 10, 10);
        lv_obj_align(cell_bars[i], LV_ALIGN_TOP_LEFT, 15 + i * cellW, 430);
        lv_bar_set_range(cell_bars[i], 3000, 3600); // typischer LiFePO4-Zellbereich in mV
        lv_bar_set_value(cell_bars[i], 3000, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(cell_bars[i], COL_ARC_GRAY, LV_PART_MAIN);
        lv_obj_set_style_bg_color(cell_bars[i], COL_GREEN, LV_PART_INDICATOR);

        cell_labels[i] = lv_label_create(lv_scr_act());
        lv_label_set_text(cell_labels[i], "-.---V");
        lv_obj_set_style_text_color(cell_labels[i], COL_TEXT_DIM, 0);
        lv_obj_set_style_text_font(cell_labels[i], &lv_font_montserrat_10, 0);
        lv_obj_align(cell_labels[i], LV_ALIGN_TOP_LEFT, 15 + i * cellW, 460);
    }
}

// ------------------------------------------------------------------
// Wisch-Geste: links = zum Verlauf, rechts = zurueck zum Hauptbildschirm
// ------------------------------------------------------------------
void screen_gesture_cb(lv_event_t* e) {
    lv_obj_t* current_screen = lv_scr_act();
    lv_indev_t* indev = lv_event_get_indev(e); // SICHERER: Holt Input direkt aus dem Event
    
    // Sicherheits-Check gegen Null-Pointer
    if (!indev) return; 

    lv_dir_t dir = lv_indev_get_gesture_dir(indev);

    if (dir == LV_DIR_LEFT && current_screen == scr_main) {
        lv_scr_load_anim(scr_history, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
        lv_indev_wait_release(indev); 
    } 
    else if (dir == LV_DIR_RIGHT && current_screen == scr_history) {
        lv_scr_load_anim(scr_main, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false);
        lv_indev_wait_release(indev); 
    }
}

// ----------------------------------------
// Labeling der x-Achse und y-achse für den zweiten Screen
// -------------------------------------------
static void chart_draw_event_cb(lv_event_t * e) {
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    
    if (!dsc || !dsc->text || dsc->text_length == 0) return; 

    // --- X-ACHSE (Zeit) ---
    if (dsc->part == LV_PART_TICKS && dsc->id == LV_CHART_AXIS_PRIMARY_X) {
        int minutesAgo = (4 - dsc->value) * 30; 
        if (minutesAgo == 0) {
            snprintf(dsc->text, dsc->text_length, "Jetzt");
        } else {
            int hours = minutesAgo / 60;
            int mins = minutesAgo % 60;
            if (hours > 0) {
                snprintf(dsc->text, dsc->text_length, "-%dh%02d", hours, mins);
            } else {
                snprintf(dsc->text, dsc->text_length, "-%dm", mins);
            }
        }
    }
    
    // --- PRIMÄRE Y-ACHSE (Links: SoC 0 bis 100 %) ---
    else if (dsc->part == LV_PART_TICKS && dsc->id == LV_CHART_AXIS_PRIMARY_Y) {
        // dsc->value gibt den genauen Achsenwert an (0 bis 100)
        snprintf(dsc->text, dsc->text_length, "%d", (int)dsc->value);
    }
    
    // --- SEKUNDÄRE Y-ACHSE (Rechts: Strom -30 bis +30 A) ---
    else if (dsc->part == LV_PART_TICKS && dsc->id == LV_CHART_AXIS_SECONDARY_Y) {
        // dsc->value gibt den Stromwert an (-30 bis +30)
        snprintf(dsc->text, dsc->text_length, "%d", (int)dsc->value);
    }
}

// ------------------------------------------------------------------
// Zweiter Screen: Verlaufsdiagramm (SoC % auf Hauptachse, Strom A auf
// Sekundaerachse, da unterschiedlicher Wertebereich/Einheit)
// ------------------------------------------------------------------
void build_history_ui() {
    // lv_scr_act() durch scr_history ersetzt!
    lv_obj_set_style_bg_color(scr_history, COL_BG, 0);

    lv_obj_t* header2 = lv_obj_create(scr_history);
    lv_obj_set_size(header2, SCREEN_W, 28);
    lv_obj_align(header2, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header2, COL_BRAND, 0);
    lv_obj_set_style_border_width(header2, 0, 0);
    lv_obj_set_style_radius(header2, 0, 0);
    lv_obj_clear_flag(header2, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title2 = lv_label_create(header2);
    lv_label_set_text(title2, "Verlauf");
    lv_obj_set_style_text_color(title2, lv_color_hex(0x1A1A1A), 0);
    lv_obj_align(title2, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_t* leg_soc = lv_label_create(scr_history);
    lv_label_set_text(leg_soc, "SoC %");
    lv_obj_set_style_text_color(leg_soc, COL_GREEN, 0);
    lv_obj_align(leg_soc, LV_ALIGN_TOP_LEFT, 15, 36);

    lv_obj_t* leg_cur = lv_label_create(scr_history);
    lv_label_set_text(leg_cur, "Strom A");
    lv_obj_set_style_text_color(leg_cur, COL_YELLOW, 0);
    lv_obj_align(leg_cur, LV_ALIGN_TOP_RIGHT, -15, 36);

    chart_history = lv_chart_create(scr_history);
    lv_obj_set_size(chart_history, SCREEN_W - 42, 370);
    lv_obj_align(chart_history, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_text_font(chart_history, &lv_font_montserrat_8, 0);
    lv_obj_set_style_pad_left(chart_history, 5, 0);
    lv_obj_set_style_pad_right(chart_history, 5, 0);
    //lv_obj_set_style_pad_top(chart_history, 10, 0);
    //lv_obj_set_style_pad_bottom(chart_history, 25, 0);
    lv_obj_set_style_bg_color(chart_history, COL_PANEL, 0);
    lv_obj_set_style_border_width(chart_history, 0, 0);
    lv_chart_set_type(chart_history, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart_history, HISTORY_SIZE);
    lv_chart_set_div_line_count(chart_history, 5, 5);
    lv_chart_set_update_mode(chart_history, LV_CHART_UPDATE_MODE_SHIFT);

    lv_chart_set_range(chart_history, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_chart_set_range(chart_history, LV_CHART_AXIS_SECONDARY_Y, -30, 30);
    lv_chart_set_axis_tick(chart_history, LV_CHART_AXIS_PRIMARY_X, 4, 2, 5, 2, true, 40);
    lv_chart_set_axis_tick(chart_history, LV_CHART_AXIS_PRIMARY_Y, 4, 2, 5, 2, true, 35);
    lv_chart_set_axis_tick(chart_history, LV_CHART_AXIS_SECONDARY_Y, 4, 2, 5, 2, true, 35);
    lv_obj_add_event_cb(chart_history, chart_draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);

    series_soc = lv_chart_add_series(chart_history, COL_GREEN, LV_CHART_AXIS_PRIMARY_Y);
    series_current = lv_chart_add_series(chart_history, COL_YELLOW, LV_CHART_AXIS_SECONDARY_Y);

    lv_obj_set_style_size(chart_history, 4, LV_PART_ITEMS);

    for (int i = 0; i < HISTORY_SIZE; i++) {
        lv_chart_set_next_value(chart_history, series_soc, LV_CHART_POINT_NONE);
        lv_chart_set_next_value(chart_history, series_current, LV_CHART_POINT_NONE);
    }

    lv_obj_t* hint = lv_label_create(scr_history);
    lv_label_set_text(hint, "<- Nach rechts wischen fuer Zurueck");
    lv_obj_set_style_text_color(hint, COL_TEXT_DIM, 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_10, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ------------------------------------------------------------------
// Wird bei jeder BMS-Aktualisierung aufgerufen, nimmt aber nur alle
// SAMPLE_INTERVAL_MS tatsaechlich einen neuen Punkt ins Diagramm auf.
// ------------------------------------------------------------------
void update_history_sample(float currentA, float socPrecise) {
    uint32_t now = millis();
    if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
    lastSampleMs = now;

    lv_chart_set_next_value(chart_history, series_soc, (int16_t)(socPrecise + 0.5f));
    lv_chart_set_next_value(chart_history, series_current, (int16_t)roundf(currentA));
}

void update_arc_mask(uint8_t soc) {
    // Grauer Bogen deckt den noch nicht erreichten Bereich ab (von aktuellem
    // SoC-Winkel bis zum Ende des 270-Grad-Bogens) - so entsteht der
    // "aufgefuellter Regenbogen"-Effekt wie in der App.
    int angle = (int)((soc / 100.0f) * 270.0f);
    if (angle < 0) angle = 0;
    if (angle > 270) angle = 270;
    lv_arc_set_bg_angles(arc_mask, angle, 270);
}

void update_ui_from_bms() {
    BmsData d;
    memcpy((void*)&d, (const void*)&currentBmsState, sizeof(BmsData));

    if (!d.isConnected) {
        lv_label_set_text(label_status, "Suche...");
        lv_obj_set_style_bg_color(header_bar, COL_YELLOW, 0);
        return;
    }

    lv_label_set_text(label_status, "Verbunden");
    lv_obj_set_style_bg_color(header_bar, d.soc <= 15 ? COL_RED : COL_BRAND, 0);

    update_arc_mask(d.soc);
    lv_label_set_text_fmt(label_soc_pct, "%.1f %%", d.socPrecise);
    lv_label_set_text_fmt(label_ah, "%.0f Ah", d.remainAh);
    lv_label_set_text(icon_center, d.isCharging ? LV_SYMBOL_CHARGE : LV_SYMBOL_BATTERY_FULL);

    lv_label_set_text_fmt(label_temp, "%d.0C", d.temp1);
    lv_label_set_text_fmt(label_power, "%.0f W", fabs(d.current * d.packVoltage));
    lv_label_set_text_fmt(label_voltage, "%.2f V", d.packVoltage);
    lv_label_set_text_fmt(label_current, "%.1f A", d.current);

    if (d.current < -0.01f) {
        lv_label_set_text_fmt(label_time, "Restlaufzeit: %dh %02dmin", d.timeHours, d.timeMinutes);
    } else if (d.current > 0.01f) {
        lv_label_set_text_fmt(label_time, "Zeit bis Voll: %dh %02dmin", d.timeHours, d.timeMinutes);
    } else {
        lv_label_set_text(label_time, "Zeit bis Voll: -h -min");
    }

    lv_label_set_text(label_charge_val, d.chargeAllowed == 1 ? "ON" : (d.chargeAllowed == 0 ? "OFF" : "--"));
    lv_obj_set_style_text_color(label_charge_val, d.chargeAllowed == 1 ? COL_GREEN : COL_TEXT_DIM, 0);

    lv_label_set_text(label_discharge_val, d.dischargeAllowed == 1 ? "ON" : (d.dischargeAllowed == 0 ? "OFF" : "--"));
    lv_obj_set_style_text_color(label_discharge_val, d.dischargeAllowed == 1 ? COL_GREEN : COL_TEXT_DIM, 0);

    // Heizung: TODO - Byte noch nicht identifiziert (siehe BLE-Task-Kommentar).
    // Bleibt bewusst "unbekannt", bis im Winter per Vergleichsmessung bestimmt.
    lv_label_set_text(label_heater_val, "unbek.");

    lv_label_set_text_fmt(label_cycles_val, "%d", d.cycles);

    lv_label_set_text_fmt(label_maxv_val, "%.3f V", d.cellMax);
    lv_label_set_text_fmt(label_minv_val, "%.3f V", d.cellMin);
    lv_label_set_text_fmt(label_diffv_val, "%.3f V", d.cellMax - d.cellMin);

    lv_label_set_text_fmt(label_cellcount, "Anzahl Zellen: %d", d.cellCount);
    for (int i = 0; i < 4; i++) {
        if (i < d.cellCount) {
            int mv = (int)(d.cellV[i] * 1000);
            lv_bar_set_value(cell_bars[i], mv, LV_ANIM_OFF);
            lv_label_set_text_fmt(cell_labels[i], "%.3fV", d.cellV[i]);
        }
    }

    update_history_sample(d.current, d.socPrecise);
}

// ==================================================================
// TEIL 5: SETUP / LOOP
// ==================================================================

uint32_t last_ui_update = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    display.init();
    display.setRotation(0); // Hochkant, natives Panel-Format 320x480
    display.setBrightness(200);
    lastTouchMs = millis();

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount fehlgeschlagen! Sounddatei nicht verfuegbar.");
    }
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(15); // 0...21

    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCREEN_W * 40);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_W;
    disp_drv.ver_res = SCREEN_H;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);

    build_ui();

    scr_main = lv_scr_act();
    scr_history = lv_obj_create(NULL);
    //lv_scr_load(scr_history);
    build_history_ui();

    lv_obj_add_event_cb(scr_main, screen_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(scr_history, screen_gesture_cb, LV_EVENT_GESTURE, NULL);

    //lv_scr_load(scr_main); // mit dem Hauptbildschirm starten

    xTaskCreatePinnedToCore(
        bleWorkerTask,
        "BLE_Task",
        10000,
        NULL,
        1,
        NULL,
        0
    );
}

void loop() {
    lv_timer_handler();
    audio.loop(); // muss regelmaessig aufgerufen werden, damit Wiedergabe laeuft

    if (triggerConnectSound) {
        triggerConnectSound = false;
        audio.connecttoFS(LittleFS, "/chime.mp3");
    }

    if (backlightOn && (millis() - lastTouchMs > SCREEN_TIMEOUT_MS)) {
        display.setBrightness(BRIGHTNESS_DIMMED);
        backlightOn = false;
    }

    if (millis() - last_ui_update > 1000) {
        update_ui_from_bms();
        last_ui_update = millis();
    }

    delay(5);
}
