#include "unihiker_k10.h"

#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <Update.h>
#include <math.h>

UNIHIKER_K10 k10;

namespace {
// ===== Experiment settings to edit before upload =====
constexpr float kDefaultPendulumLengthM = 0.420f;  // pivot to bob center, meters
constexpr int kPivotX = 160;                // pivot pixel x in the HUSKYLENS image
constexpr int kPivotY = 24;                 // pivot pixel y in the HUSKYLENS image
constexpr uint16_t kTargetId = 1;           // learned HUSKYLENS color ID for the bob
constexpr uint16_t kIgnoreFrameId = 2;      // learned large frame/background color ID
constexpr char kApSsid[] = "k10-pendulum";
constexpr char kApPassword[] = "12345678";
constexpr char kDefaultStaSsid[] = "DFRobot-guest";
constexpr char kDefaultStaPassword[] = "dfrobot@2017";

// ===== Capture settings =====
constexpr uint8_t kScreenDir = 2;
constexpr uint16_t kMaxSamples = 200;
constexpr uint16_t kMinValidSamples = 24;
constexpr float kMinCaptureFps = 2.0f;
constexpr float kMaxThetaRad = 1.5f;
constexpr float kMinReasonableG = 5.0f;
constexpr float kMaxReasonableG = 15.0f;
constexpr float kMinFitCycles = 1.4f;
constexpr float kMaxFitRmseRatio = 0.35f;
constexpr float kMinFitRmsePx = 8.0f;
constexpr uint32_t kHudIntervalMs = 1000;
constexpr uint32_t kHuskyPollIntervalMs = 25;
constexpr uint32_t kHuskyReadTimeoutMs = 10;

// ===== HUSKYLENS protocol =====
constexpr uint8_t kHuskyI2cAddress = 0x50;
constexpr uint8_t kCommandGetResult = 0x01;
constexpr uint8_t kCommandSetAlgorithm = 0x0A;
constexpr uint8_t kCommandKnock = 0x00;
constexpr uint8_t kCommandReturnArgs = 0x1A;
constexpr uint8_t kCommandReturnInfo = 0x1B;
constexpr uint8_t kCommandReturnBlock = 0x1C;
constexpr uint8_t kAlgorithmColorRecognition = 0x04;
constexpr uint8_t kAlgorithmAny = 0x00;
constexpr uint8_t kBoardLargeRam = 0x01;
constexpr uint8_t kMaxHuskyFrameBytes = 96;
constexpr uint8_t kHuskyReadChunkBytes = 16;

struct AnalysisResult {
    bool ok = false;
    uint16_t valid = 0;
    float x0 = 0.0f;
    float y0 = 0.0f;
    float amplitudePx = 0.0f;
    float minorAmplitudePx = 0.0f;
    float fitRmsePx = 0.0f;
    float thetaRad = 0.0f;
    float frequencyHz = 0.0f;
    float periodS = 0.0f;
    float g = 0.0f;
    float captureFps = 0.0f;
    uint8_t qualityCode = 1;
};

uint32_t lastHudMs = 0;
uint32_t lastPollMs = 0;
uint32_t lastTargetMs = 0;
uint32_t captureStartMs = 0;
uint16_t sampleCount = 0;
bool capturing = false;
bool huskyReady = false;
uint16_t lostFrames = 0;
uint16_t lastSeenBlockCount = 0;
uint16_t lastMatchedId = 0;
uint16_t lastBlockWidth = 0;
uint16_t lastBlockHeight = 0;
float lastX = 0.0f;
float lastY = 0.0f;
bool lastHuskyReadOk = false;
uint8_t lastHuskyCommand = 0;
uint8_t lastHuskyDataLength = 0;
uint8_t lastHuskyError = 0;
uint8_t huskyRx[kMaxHuskyFrameBytes];
uint8_t huskyRxIndex = 0;
float pendulumLengthM = kDefaultPendulumLengthM;
String staSsid = kDefaultStaSsid;
String staPassword = kDefaultStaPassword;

float sampleT[kMaxSamples];
float sampleX[kMaxSamples];
float sampleY[kMaxSamples];
AnalysisResult lastAnalysis;
bool calibrated = false;
bool calibrating = false;
float referenceG = 9.80f;
bool otaRestartPending = false;
uint32_t otaRestartAtMs = 0;
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

void startCapture();
void stopAndAnalyze();

void drawTextLine(const String &text, int16_t y, uint32_t color = 0x00FF00) {
    k10.canvas->canvasText(text, 4, y, color, k10.canvas->eCNAndENFont16, 150, true);
}

void clearScreen() {
    k10.setScreenBackground(0x000000);
    k10.canvas->canvasClear();
}

void drawHud() {
    drawTextLine("Color pendulum", 6);
    drawTextLine(capturing ? "A:capturing  B:result" : "A:start  B:result", 30);
    drawTextLine(String("samples: ") + String(sampleCount), 54);
    drawTextLine(huskyReady ? "I2C:ok" : "I2C:check", 78, huskyReady ? 0x00FF00 : 0xFF0000);
    if (lastTargetMs > 0) {
        drawTextLine(String("x,y=") + String(lastX, 0) + "," + String(lastY, 0), 102, 0x00FFFF);
        drawTextLine(String("lost: ") + String(lostFrames), 126, lostFrames == 0 ? 0x00FF00 : 0xFFFF00);
    }
    const IPAddress ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP();
    drawTextLine(String("web: ") + ip.toString(), 202, 0x00FFFF);
    k10.canvas->updateCanvas();
}

uint16_t readLe16(const uint8_t *data) {
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

uint8_t checksum(const uint8_t *data, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum & 0xFF;
}

bool sendHuskyCommand(uint8_t command, uint8_t algorithm, const uint8_t *data = nullptr, uint8_t dataLength = 0) {
    huskyRxIndex = 0;
    uint8_t frame[kMaxHuskyFrameBytes] = {0x55, 0xAA, command, algorithm, dataLength};
    for (uint8_t i = 0; i < dataLength; ++i) {
        frame[5 + i] = data[i];
    }
    const uint8_t frameLength = 5 + dataLength + 1;
    frame[frameLength - 1] = checksum(frame, frameLength - 1);

    Wire.beginTransmission(kHuskyI2cAddress);
    Wire.write(frame, frameLength);
    return Wire.endTransmission() == 0;
}

bool parseHuskyByte(uint8_t byte, uint8_t &command, uint8_t *data, uint8_t &dataLength) {
    if (huskyRxIndex == 0 && byte != 0x55) {
        return false;
    }
    if (huskyRxIndex == 1 && byte != 0xAA) {
        huskyRxIndex = 0;
        return false;
    }
    if (huskyRxIndex >= kMaxHuskyFrameBytes) {
        huskyRxIndex = 0;
        lastHuskyError = 4;
        return false;
    }

    huskyRx[huskyRxIndex++] = byte;
    if (huskyRxIndex < 5) {
        return false;
    }

    const uint8_t len = huskyRx[4];
    const uint8_t frameLength = 5 + len + 1;
    if (frameLength > kMaxHuskyFrameBytes) {
        huskyRxIndex = 0;
        lastHuskyError = 4;
        return false;
    }
    if (huskyRxIndex < frameLength) {
        return false;
    }

    if (checksum(huskyRx, frameLength - 1) != huskyRx[frameLength - 1]) {
        huskyRxIndex = 0;
        lastHuskyError = 2;
        return false;
    }

    command = huskyRx[2];
    dataLength = len;
    for (uint8_t i = 0; i < len; ++i) {
        data[i] = huskyRx[5 + i];
    }
    huskyRxIndex = 0;
    lastHuskyCommand = command;
    lastHuskyDataLength = dataLength;
    lastHuskyError = 0;
    return true;
}

bool readHuskyFrame(uint8_t &command, uint8_t *data, uint8_t &dataLength, uint32_t timeoutMs = kHuskyReadTimeoutMs) {
    const uint32_t deadline = millis() + timeoutMs;
    bool sawByte = huskyRxIndex > 0;
    while (int32_t(deadline - millis()) > 0) {
        Wire.requestFrom(kHuskyI2cAddress, kHuskyReadChunkBytes);
        while (Wire.available()) {
            sawByte = true;
            if (parseHuskyByte(Wire.read(), command, data, dataLength)) {
                return true;
            }
        }
        delay(1);
    }
    lastHuskyError = sawByte ? 3 : 1;
    return false;
}

bool readArgsOk(uint32_t timeoutMs = kHuskyReadTimeoutMs) {
    uint8_t command = 0;
    uint8_t data[kMaxHuskyFrameBytes] = {};
    uint8_t dataLength = 0;
    return readHuskyFrame(command, data, dataLength, timeoutMs) && command == kCommandReturnArgs && dataLength > 1 && data[1] == 0;
}

bool initHuskyLens() {
    uint8_t knockData[10] = {kBoardLargeRam, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!sendHuskyCommand(kCommandKnock, kAlgorithmAny, knockData, sizeof(knockData)) || !readArgsOk(250)) {
        return false;
    }

    uint8_t algorithmData[10] = {kAlgorithmColorRecognition, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!sendHuskyCommand(kCommandSetAlgorithm, kAlgorithmAny, algorithmData, sizeof(algorithmData))) {
        return false;
    }
    return readArgsOk(250);
}

bool readBlockResponse(float &cx, float &cy) {
    uint8_t command = 0;
    uint8_t data[kMaxHuskyFrameBytes] = {};
    uint8_t dataLength = 0;
    lastHuskyReadOk = false;

    const uint32_t infoDeadline = millis() + 15;
    bool gotInfo = false;
    while (int32_t(infoDeadline - millis()) > 0) {
        if (!readHuskyFrame(command, data, dataLength, kHuskyReadTimeoutMs)) {
            continue;
        }
        if (command == kCommandReturnInfo && dataLength >= 10) {
            gotInfo = true;
            break;
        }
    }
    if (!gotInfo) {
        return false;
    }

    const uint16_t blockCount = readLe16(&data[6]);
    lastSeenBlockCount = blockCount;
    uint16_t blocksRead = 0;
    const uint32_t blockDeadline = millis() + 40;
    while (blocksRead < blockCount && int32_t(blockDeadline - millis()) > 0) {
        if (!readHuskyFrame(command, data, dataLength, kHuskyReadTimeoutMs)) {
            continue;
        }
        if (command != kCommandReturnBlock || dataLength < 10) {
            continue;
        }
        blocksRead++;

        const uint16_t id = data[0];
        const uint16_t width = readLe16(&data[6]);
        const uint16_t height = readLe16(&data[8]);
        lastMatchedId = id;
        if (id == kIgnoreFrameId) {
            continue;
        }
        if (id == kTargetId) {
            cx = readLe16(&data[2]);
            cy = readLe16(&data[4]);
            lastBlockWidth = width;
            lastBlockHeight = height;
            lastHuskyReadOk = true;
            return true;
        }
    }
    lastHuskyReadOk = false;
    return false;
}

bool readBobCenter(float &cx, float &cy) {
    return sendHuskyCommand(kCommandGetResult, kAlgorithmColorRecognition) && readBlockResponse(cx, cy);
}

struct EllipseFit {
    bool ok = false;
    float frequencyHz = 0.0f;
    float x0 = 0.0f;
    float y0 = 0.0f;
    float xCos = 0.0f;
    float xSin = 0.0f;
    float yCos = 0.0f;
    float ySin = 0.0f;
    float majorPx = 0.0f;
    float minorPx = 0.0f;
    float rmsePx = 0.0f;
};

bool solve3x3(float a[3][3], float b[3], float out[3]) {
    for (uint8_t col = 0; col < 3; ++col) {
        uint8_t pivot = col;
        float best = fabsf(a[col][col]);
        for (uint8_t row = col + 1; row < 3; ++row) {
            const float v = fabsf(a[row][col]);
            if (v > best) {
                best = v;
                pivot = row;
            }
        }
        if (best < 1e-6f) {
            return false;
        }
        if (pivot != col) {
            for (uint8_t c = col; c < 3; ++c) {
                const float tmp = a[col][c];
                a[col][c] = a[pivot][c];
                a[pivot][c] = tmp;
            }
            const float tmpB = b[col];
            b[col] = b[pivot];
            b[pivot] = tmpB;
        }
        const float div = a[col][col];
        for (uint8_t c = col; c < 3; ++c) {
            a[col][c] /= div;
        }
        b[col] /= div;
        for (uint8_t row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }
            const float factor = a[row][col];
            for (uint8_t c = col; c < 3; ++c) {
                a[row][c] -= factor * a[col][c];
            }
            b[row] -= factor * b[col];
        }
    }
    out[0] = b[0];
    out[1] = b[1];
    out[2] = b[2];
    return true;
}

bool fitAtFrequency(float frequencyHz, EllipseFit &fit, float &sse) {
    float normal[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    float rhsX[3] = {0.0f, 0.0f, 0.0f};
    float rhsY[3] = {0.0f, 0.0f, 0.0f};
    const float omega = 2.0f * PI * frequencyHz;

    for (uint16_t i = 0; i < sampleCount; ++i) {
        const float basis[3] = {1.0f, cosf(omega * sampleT[i]), sinf(omega * sampleT[i])};
        for (uint8_t r = 0; r < 3; ++r) {
            rhsX[r] += basis[r] * sampleX[i];
            rhsY[r] += basis[r] * sampleY[i];
            for (uint8_t c = r; c < 3; ++c) {
                normal[r][c] += basis[r] * basis[c];
            }
        }
    }
    normal[1][0] = normal[0][1];
    normal[2][0] = normal[0][2];
    normal[2][1] = normal[1][2];

    float nx[3][3];
    float ny[3][3];
    float bx[3];
    float by[3];
    for (uint8_t r = 0; r < 3; ++r) {
        bx[r] = rhsX[r];
        by[r] = rhsY[r];
        for (uint8_t c = 0; c < 3; ++c) {
            nx[r][c] = normal[r][c];
            ny[r][c] = normal[r][c];
        }
    }

    float cx[3];
    float cy[3];
    if (!solve3x3(nx, bx, cx) || !solve3x3(ny, by, cy)) {
        return false;
    }

    sse = 0.0f;
    for (uint16_t i = 0; i < sampleCount; ++i) {
        const float c = cosf(omega * sampleT[i]);
        const float s = sinf(omega * sampleT[i]);
        const float rx = sampleX[i] - (cx[0] + cx[1] * c + cx[2] * s);
        const float ry = sampleY[i] - (cy[0] + cy[1] * c + cy[2] * s);
        sse += rx * rx + ry * ry;
    }

    fit.ok = true;
    fit.frequencyHz = frequencyHz;
    fit.x0 = cx[0];
    fit.xCos = cx[1];
    fit.xSin = cx[2];
    fit.y0 = cy[0];
    fit.yCos = cy[1];
    fit.ySin = cy[2];
    fit.rmsePx = sqrtf(sse / max(1.0f, 2.0f * sampleCount));

    const float aa = fit.xCos * fit.xCos + fit.xSin * fit.xSin;
    const float bb = fit.xCos * fit.yCos + fit.xSin * fit.ySin;
    const float cc = fit.yCos * fit.yCos + fit.ySin * fit.ySin;
    const float trace = aa + cc;
    const float disc = sqrtf(max(0.0f, (aa - cc) * (aa - cc) + 4.0f * bb * bb));
    fit.majorPx = sqrtf(max(0.0f, 0.5f * (trace + disc)));
    fit.minorPx = sqrtf(max(0.0f, 0.5f * (trace - disc)));
    return true;
}

EllipseFit fitEllipseMotion(float minFrequencyHz, float maxFrequencyHz) {
    EllipseFit bestFit;
    float bestSse = 1.0e30f;
    float bestFreq = 0.0f;
    const uint8_t coarseSteps = 72;
    const uint8_t fineSteps = 72;
    const float span = maxFrequencyHz - minFrequencyHz;

    for (uint8_t i = 0; i <= coarseSteps; ++i) {
        const float f = minFrequencyHz + span * i / coarseSteps;
        EllipseFit candidate;
        float sse = 0.0f;
        if (fitAtFrequency(f, candidate, sse) && sse < bestSse) {
            bestSse = sse;
            bestFit = candidate;
            bestFreq = f;
        }
    }

    if (!bestFit.ok) {
        return bestFit;
    }

    const float fineHalfSpan = span / coarseSteps;
    const float fineMin = max(minFrequencyHz, bestFreq - fineHalfSpan);
    const float fineMax = min(maxFrequencyHz, bestFreq + fineHalfSpan);
    for (uint8_t i = 0; i <= fineSteps; ++i) {
        const float f = fineMin + (fineMax - fineMin) * i / fineSteps;
        EllipseFit candidate;
        float sse = 0.0f;
        if (fitAtFrequency(f, candidate, sse) && sse < bestSse) {
            bestSse = sse;
            bestFit = candidate;
        }
    }

    return bestFit;
}

AnalysisResult analyzeSamples() {
    AnalysisResult result;
    result.valid = sampleCount;
    if (sampleCount < kMinValidSamples) {
        result.qualityCode = 1;
        return result;
    }

    const float captureTime = sampleT[sampleCount - 1] - sampleT[0];
    result.captureFps = captureTime > 0.0f ? (sampleCount - 1) / captureTime : 0.0f;
    if (result.captureFps < kMinCaptureFps) {
        result.qualityCode = 2;
        return result;
    }

    float sumLengthPx = 0.0f;

    for (uint16_t i = 0; i < sampleCount; ++i) {
        const float x = sampleX[i];
        const float y = sampleY[i];
        const float dx = x - kPivotX;
        const float dy = y - kPivotY;
        sumLengthPx += sqrtf(dx * dx + dy * dy);
    }

    const float minFrequencyHz = sqrtf(kMinReasonableG / pendulumLengthM) / (2.0f * PI);
    const float maxFrequencyHz = sqrtf(kMaxReasonableG / pendulumLengthM) / (2.0f * PI);
    const EllipseFit fit = fitEllipseMotion(minFrequencyHz, maxFrequencyHz);
    if (!fit.ok || fit.frequencyHz <= 0.0f) {
        result.qualityCode = 3;
        return result;
    }

    result.x0 = fit.x0;
    result.y0 = fit.y0;
    result.amplitudePx = fit.majorPx;
    result.minorAmplitudePx = fit.minorPx;
    result.fitRmsePx = fit.rmsePx;
    result.frequencyHz = fit.frequencyHz;
    result.periodS = 1.0f / result.frequencyHz;

    if (captureTime < result.periodS * kMinFitCycles) {
        result.qualityCode = 7;
        return result;
    }
    if (result.fitRmsePx > max(kMinFitRmsePx, result.amplitudePx * kMaxFitRmseRatio)) {
        result.qualityCode = 6;
        return result;
    }

    const float lengthPx = sumLengthPx / sampleCount;
    result.thetaRad = asinf(min(0.999f, result.amplitudePx / max(1.0f, lengthPx)));
    const float omega = 2.0f * PI * result.frequencyHz;

    if (calibrating) {
        pendulumLengthM = referenceG / (omega * omega);
        preferences.putFloat("length_m", pendulumLengthM);
        preferences.putBool("calibrated", true);
        calibrated = true;
        calibrating = false;
        result.g = referenceG;
        result.ok = true;
        result.qualityCode = 0;
        return result;
    }
    result.g = omega * omega * pendulumLengthM;
    if (result.thetaRad > kMaxThetaRad) {
        result.qualityCode = 4;
        result.ok = true;
        return result;
    }
    if (result.g < kMinReasonableG || result.g > kMaxReasonableG) {
        result.qualityCode = 5;
        return result;
    }
    result.ok = true;
    result.qualityCode = 0;
    return result;
}

void appendJsonNumber(String &out, const char *key, float value, uint8_t digits, bool comma = true) {
    out += "\"";
    out += key;
    out += "\":";
    out += String(value, static_cast<unsigned int>(digits));
    if (comma) {
        out += ",";
    }
}

void appendJsonString(String &out, const char *key, const String &value, bool comma = true) {
    out += "\"";
    out += key;
    out += "\":\"";
    for (uint16_t i = 0; i < value.length(); ++i) {
        const char c = value[i];
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += c;
    }
    out += "\"";
    if (comma) {
        out += ",";
    }
}

void appendJsonUInt(String &out, const char *key, uint32_t value, bool comma = true) {
    out += "\"";
    out += key;
    out += "\":";
    out += String(value);
    if (comma) {
        out += ",";
    }
}

String analysisJson(const AnalysisResult &result) {
    String out = "{";
    out.reserve(320);
    out += "\"ok\":";
    out += result.ok ? "true," : "false,";
    appendJsonUInt(out, "valid", result.valid);
    appendJsonNumber(out, "x0_px", result.x0, 3);
    appendJsonNumber(out, "y0_px", result.y0, 3);
    appendJsonNumber(out, "amplitude_px", result.amplitudePx, 3);
    appendJsonNumber(out, "minor_amplitude_px", result.minorAmplitudePx, 3);
    appendJsonNumber(out, "fit_rmse_px", result.fitRmsePx, 3);
    appendJsonNumber(out, "theta_rad", result.thetaRad, 5);
    appendJsonNumber(out, "theta_deg", result.thetaRad * 180.0f / PI, 3);
    appendJsonNumber(out, "frequency_hz", result.frequencyHz, 5);
    appendJsonNumber(out, "period_s", result.periodS, 5);
    appendJsonNumber(out, "g_m_s2", result.g, 5);
    appendJsonNumber(out, "capture_fps", result.captureFps, 2);
    appendJsonUInt(out, "quality_code", result.qualityCode, false);
    out += "}";
    return out;
}

void loadSettings() {
    preferences.begin("pendulum", true);
    pendulumLengthM = preferences.getFloat("length_m", kDefaultPendulumLengthM);
    if (pendulumLengthM < 0.05f || pendulumLengthM > 5.0f) {
        pendulumLengthM = kDefaultPendulumLengthM;
    }
    staSsid = preferences.getString("sta_ssid", kDefaultStaSsid);
    staPassword = preferences.getString("sta_pass", kDefaultStaPassword);
    calibrated = preferences.getBool("calibrated", false);
    referenceG = preferences.getFloat("ref_g", 9.80f);
}

void saveExperimentSettings() {
    preferences.putFloat("length_m", pendulumLengthM);
    preferences.putFloat("ref_g", referenceG);
}

void saveWifiSettings() {
    preferences.putString("sta_ssid", staSsid);
    preferences.putString("sta_pass", staPassword);
}

void connectSta() {
    if (staSsid.length() == 0) {
        return;
    }
    WiFi.begin(staSsid.c_str(), staPassword.c_str());
}

String wifiStatusText() {
    switch (WiFi.status()) {
        case WL_CONNECTED:
            return "connected";
        case WL_CONNECT_FAILED:
            return "failed";
        case WL_NO_SSID_AVAIL:
            return "ssid_not_found";
        case WL_DISCONNECTED:
            return "disconnected";
        default:
            return "connecting";
    }
}

void handleRoot() {
    static const char html[] PROGMEM = R"HTML(
<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>K10 单摆</title>
<style>
:root{font-family:system-ui,sans-serif;color:#17202a;background:#f6f7f9}
body{margin:0}main{max-width:800px;margin:0 auto;padding:18px}
h1{font-size:24px;margin:0}h2{font-size:18px;margin:18px 0 8px}
.actions{display:flex;gap:8px;flex-wrap:wrap}button,a.button{border:1px solid #0f766e;background:#0f766e;color:white;border-radius:6px;padding:9px 12px;text-decoration:none;font-size:14px;cursor:pointer}
button.secondary,a.secondary{background:white;color:#0f766e}button:disabled{opacity:.5}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-top:14px}
.metric{background:white;border:1px solid #d7dde4;border-radius:8px;padding:10px}.label{font-size:12px;color:#5b6775}.value{font-size:22px;font-weight:700;margin-top:4px}
table{border-collapse:collapse;width:100%;background:white;border:1px solid #d7dde4}th,td{padding:7px 8px;border-bottom:1px solid #e7ebef;text-align:right}th:first-child,td:first-child{text-align:left}th{background:#edf2f7;font-size:13px}td{font-variant-numeric:tabular-nums}
.panel{margin-top:14px}.status{font-weight:700}.ok{color:#11805a}.bad{color:#c2410c}.muted{color:#5b6775}.box{margin-top:18px;background:white;border:1px solid #d7dde4;border-radius:8px;padding:12px}
input{max-width:100%;box-sizing:border-box;border:1px solid #bcc7d3;border-radius:6px;padding:8px;font-size:14px}label{display:block;font-size:13px;color:#5b6775;margin:8px 0 4px}.formgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px;align-items:end}
</style></head><body><main>
<header><div><h1>K10 单摆</h1><div class="muted" id="calibStatus"></div></div>
<div class="actions"><button id="btnStart" onclick="startCapture()">开始采样</button><a class="button secondary" href="/samples.csv">下载 CSV</a></div></header>
<section class="grid" id="metrics"></section>
<section class="box" id="calibBox" style="display:none"><h2>首次校准</h2><div class="muted">选择地点后自动填入标准重力加速度，点击校准自动反推摆长</div><div class="formgrid" style="margin-top:8px"><div><label>地点</label><select id="loc" onchange="setLoc()"><option value="9.794">上海 9.794</option><option value="9.801">北京 9.801</option><option value="9.788">广州 9.788</option><option value="9.80" selected>其他</option></select></div><div><label>标准 g / m/s²</label><input id="refG" type="number" min="9.7" max="9.9" step="0.001" value="9.80"></div><button onclick="calibrate()">开始校准</button></div></section>
<section class="box"><h2>摆长</h2><div class="muted">校准后自动填入，也可手动修改</div><div class="formgrid" style="margin-top:8px"><div><label>摆长 L / m</label><input id="length" type="number" min="0.05" max="5" step="0.001"></div><button onclick="saveLength()">保存</button></div></section>
<section class="panel"><h2>计算结果</h2><table><tbody id="result"></tbody></table></section>
<section class="panel"><h2>运动轨迹</h2><canvas id="plot" style="width:100%;max-width:600px;height:auto;border:1px solid #d7dde4;border-radius:6px"></canvas></section>
<section class="panel"><h2>最近样本</h2><table><thead><tr><th>#</th><th>t/s</th><th>x/px</th><th>y/px</th></tr></thead><tbody id="samples"></tbody></table></section>
<section class="box"><h2>配网</h2><div class="muted" id="net"></div><div class="formgrid"><div><label>WiFi SSID</label><input id="ssid"></div><div><label>WiFi 密码</label><input id="pass" type="password"></div><button onclick="saveWifi()">保存并连接</button></div></section>
<section class="box"><h2>OTA 更新</h2><form method="POST" action="/ota" enctype="multipart/form-data"><input type="file" name="firmware" required> <button type="submit">上传固件</button></form></section>
</main><script>
const fmt=(v,d=3)=>Number.isFinite(v)?Number(v).toFixed(d):'--';
const qualityText=q=>['有效','样本不足','采样帧率过低','无法估算周期','摆角过大','g 超出合理范围','椭圆拟合误差过大','采样时长不足'][q]||'数据异常';
async function post(path){await fetch(path,{method:'POST'});await load();}
async function startCapture(){allSamples=[];lastIdx=-1;await post('/start');}
function setLoc(){document.getElementById('refG').value=document.getElementById('loc').value;}
async function recalibrate(){await fetch('/config',{method:'POST',body:new URLSearchParams({reset_calib:'1'})});await load();}
async function calibrate(){
 const g=document.getElementById('refG').value;
 const body=new URLSearchParams();body.set('reference_g',g);
 await fetch('/config',{method:'POST',body});
 await post('/start');
}
async function saveLength(){
 const body=new URLSearchParams();
 body.set('length_m',document.getElementById('length').value);
 await fetch('/config',{method:'POST',body});
 await load();
}
async function saveWifi(){
 const body=new URLSearchParams();
 body.set('ssid',document.getElementById('ssid').value);
 body.set('password',document.getElementById('pass').value);
 await fetch('/config',{method:'POST',body});
 await load();
}
function row(k,v){return `<tr><td>${k}</td><td>${v}</td></tr>`}
async function load(){
 const d=await (await fetch('/data.json',{cache:'no-store'})).json();
 const a=d.analysis||{};
 const calc=(v,d=3)=>a.ok?fmt(v,d):'--';
 if(document.activeElement.id!=='length')document.getElementById('length').value=fmt(d.length_m,3);
 if(document.activeElement.id!=='ssid')document.getElementById('ssid').value=d.staSsid||'';
 document.getElementById('calibStatus').innerHTML=d.calibrated?'已校准 <a href="#" onclick="recalibrate()" style="color:#0f766e;font-size:13px">重新校准</a>':'未校准';
 document.getElementById('calibBox').style.display=d.calibrated?'none':'block';
 document.getElementById('net').textContent=`AP: ${d.apIp} / 局域网: ${d.staIp||'未连接'} / 状态: ${d.wifiStatus}`;
  const remain=Math.max(0,Math.ceil(60-d.captureTime));
  const statusText=d.capturing?`采样中 ${remain}s`:(a.ok?'已计算':'待采样');
  document.getElementById('btnStart').disabled=d.capturing;
  document.getElementById('btnStart').textContent=d.capturing?`剩余 ${remain}s`:'开始采样';
  document.getElementById('metrics').innerHTML=[
   ['状态',`<span class="status ${d.huskyReady?'ok':'bad'}">${statusText}</span>`],
   ['样本数',d.sampleCount],
   ['采样时间',`${fmt(d.captureTime,1)} s`],
   ['最后坐标',`${fmt(d.lastX,1)}, ${fmt(d.lastY,1)}`],
   ['摆长 L',`${fmt(d.length_m,3)} m`]
  ].map(m=>`<div class="metric"><div class="label">${m[0]}</div><div class="value">${m[1]}</div></div>`).join('');
 document.getElementById('result').innerHTML=[
  row('数据质量',qualityText(a.quality_code)),
  row('周期 T',`${calc(a.period_s,5)} s`),
  row('频率 f',`${calc(a.frequency_hz,5)} Hz`),
  row('重力加速度 g',`${calc(a.g_m_s2,5)} m/s²`),
  row('标准值',`${fmt(d.refG,4)} m/s²`),
  row('误差',d.analysis.ok?`${((a.g_m_s2-d.refG)/d.refG*100).toFixed(2)}%`:'--'),
  row('有效帧率',`${fmt(a.capture_fps,2)} fps`)
 ].join('');
  document.getElementById('samples').innerHTML=d.samples.map(s=>`<tr><td>${s.i}</td><td>${fmt(s.t,3)}</td><td>${fmt(s.x,1)}</td><td>${fmt(s.y,1)}</td></tr>`).join('');
  drawPlot(d);
 }
let allSamples=[],lastIdx=-1;
function drawPlot(d){
 const pts=d.samples||[];
 if(!d.capturing&&!d.analysis.ok&&pts.length===0){allSamples=[];lastIdx=-1;}
 if(d.capturing||d.analysis.ok){
  for(const s of pts){if(s.i>lastIdx){allSamples.push(s);lastIdx=s.i;}}
 }
 if(allSamples.length<2)return;
 const c=document.getElementById('plot'),w=c.width=600,h=c.height=360;
 const ctx=c.getContext('2d');
 const xs=allSamples.map(s=>s.x),ys=allSamples.map(s=>s.y);
 const pad=20,xMin=Math.min(...xs)-10,xMax=Math.max(...xs)+10,yMin=Math.min(...ys)-10,yMax=Math.max(...ys)+10;
 const sx=x=>w-pad-(x-xMin)/(xMax-xMin)*(w-2*pad);
 const sy=y=>h-pad-(y-yMin)/(yMax-yMin)*(h-2*pad);
 ctx.clearRect(0,0,w,h);
 ctx.strokeStyle='#e0e0e0';ctx.lineWidth=.5;
 for(let i=0;i<=4;i++){const x=pad+(w-2*pad)*i/4;ctx.beginPath();ctx.moveTo(x,pad);ctx.lineTo(x,h-pad);ctx.stroke();}
 for(let i=0;i<=3;i++){const y=pad+(h-2*pad)*i/3;ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(w-pad,y);ctx.stroke();}
 ctx.fillStyle='#999';ctx.font='10px system-ui';
 ctx.fillText(Math.round(xMin),pad,pad-4);ctx.fillText(Math.round(xMax),w-pad-20,pad-4);
 ctx.fillText(Math.round(yMin),pad+2,h-pad+12);ctx.fillText(Math.round(yMax),pad+2,pad+12);
 for(let i=0;i<allSamples.length;i++){
  const t=i/allSamples.length,px=sx(allSamples[i].x),py=sy(allSamples[i].y);
  ctx.fillStyle=`rgb(${Math.round(41+214*t)},${Math.round(128-86*t)},${Math.round(255-214*t)})`;
  ctx.beginPath();ctx.arc(px,py,3,0,2*Math.PI);ctx.fill();
 }
 const px0=sx(d.pivotX||0),py0=sy(d.pivotY||0);
 ctx.strokeStyle='#c2410c';ctx.lineWidth=2;
 ctx.beginPath();ctx.moveTo(px0-8,py0);ctx.lineTo(px0+8,py0);ctx.stroke();
 ctx.beginPath();ctx.moveTo(px0,py0-8);ctx.lineTo(px0,py0+8);ctx.stroke();
 if(d.analysis&&d.analysis.ok){
  const a=d.analysis,g=a.g_m_s2,T=a.period_s,f=a.frequency_hz,A=a.amplitude_px;
  ctx.fillStyle='rgba(0,0,0,.7)';ctx.fillRect(pad,pad,260,100);
  ctx.fillStyle='#fff';ctx.font='bold 13px system-ui';
  ctx.fillText(`T=${fmt(T,4)} s  f=${fmt(f,4)} Hz`,pad+8,pad+18);
  const errPct=(g-d.refG)/d.refG*100;
  ctx.fillText(`g=${fmt(g,4)} m/s²  L=${fmt(d.length_m,3)} m`,pad+8,pad+38);
  ctx.fillText(`标准 ${fmt(d.refG,4)}  误差 ${errPct.toFixed(2)}%`,pad+8,pad+58);
  ctx.fillText(`θ=${fmt(a.theta_deg,1)}°  RMSE=${fmt(a.fit_rmse_px,1)} px`,pad+8,pad+76);
 }
}
load();setInterval(load,1000);
</script></body></html>
)HTML";
    server.send_P(200, "text/html; charset=utf-8", html);
}

void handleDataJson() {
    String out = "{";
    out.reserve(4096);
    out += "\"capturing\":";
    out += capturing ? "true," : "false,";
    out += "\"huskyReady\":";
    out += huskyReady ? "true," : "false,";
    appendJsonUInt(out, "sampleCount", sampleCount);
    const float captureTime = capturing ? (millis() - captureStartMs) / 1000.0f : (sampleCount > 0 ? sampleT[sampleCount - 1] - sampleT[0] : 0.0f);
    appendJsonNumber(out, "captureTime", captureTime, 1);
    appendJsonNumber(out, "lastX", lastX, 1);
    appendJsonNumber(out, "lastY", lastY, 1);
    appendJsonNumber(out, "length_m", pendulumLengthM, 3);
    out += "\"calibrated\":";
    out += calibrated ? "true," : "false,";
    appendJsonNumber(out, "refG", referenceG, 3);
    appendJsonUInt(out, "pivotX", kPivotX);
    appendJsonUInt(out, "pivotY", kPivotY);
    appendJsonString(out, "apIp", WiFi.softAPIP().toString());
    appendJsonString(out, "staIp", WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String(""));
    appendJsonString(out, "wifiStatus", wifiStatusText());
    appendJsonString(out, "staSsid", staSsid);
    out += "\"analysis\":";
    out += analysisJson(lastAnalysis);
    out += ",\"samples\":[";
    const uint16_t start = sampleCount > 3 ? sampleCount - 3 : 0;
    for (uint16_t i = start; i < sampleCount; ++i) {
        if (i > start) {
            out += ",";
        }
        out += "{\"i\":";
        out += String(i);
        out += ",\"t\":";
        out += String(sampleT[i] - sampleT[0], 3);
        out += ",\"x\":";
        out += String(sampleX[i], 1);
        out += ",\"y\":";
        out += String(sampleY[i], 1);
        out += "}";
    }
    out += "]}";
    server.send(200, "application/json", out);
}

void handleSamplesCsv() {
    String out = "index,time_s,x_px,y_px\n";
    out.reserve(64 + sampleCount * 28);
    for (uint16_t i = 0; i < sampleCount; ++i) {
        out += String(i);
        out += ",";
        out += String(sampleT[i] - sampleT[0], 4);
        out += ",";
        out += String(sampleX[i], 2);
        out += ",";
        out += String(sampleY[i], 2);
        out += "\n";
    }
    server.send(200, "text/csv", out);
}

void handleConfig() {
    if (server.hasArg("reset_calib")) {
        calibrated = false;
        preferences.putBool("calibrated", false);
    }
    if (server.hasArg("reference_g")) {
        referenceG = server.arg("reference_g").toFloat();
        preferences.putFloat("ref_g", referenceG);
        calibrating = true;
    }
    if (server.hasArg("length_m")) {
        const float length = server.arg("length_m").toFloat();
        if (length >= 0.05f && length <= 5.0f) {
            pendulumLengthM = length;
            saveExperimentSettings();
        }
    }

    if (server.hasArg("ssid")) {
        staSsid = server.arg("ssid");
        if (server.hasArg("password")) {
            staPassword = server.arg("password");
        }
        saveWifiSettings();
        WiFi.disconnect(false, false);
        delay(100);
        connectSta();
    }

    server.send(200, "application/json", "{\"ok\":true}");
}

void handleHuskyTest() {
    float x = 0.0f;
    float y = 0.0f;
    const bool ok = huskyReady && readBobCenter(x, y);
    if (ok) {
        lastX = x;
        lastY = y;
        lastTargetMs = millis();
    }
    Serial.print("husky_test=");
    Serial.print(ok ? "ok" : "fail");
    Serial.print(" blocks=");
    Serial.print(lastSeenBlockCount);
    Serial.print(" id=");
    Serial.print(lastMatchedId);
    Serial.print(" x=");
    Serial.print(lastX, 1);
    Serial.print(" y=");
    Serial.print(lastY, 1);
    Serial.print(" cmd=0x");
    Serial.print(lastHuskyCommand, HEX);
    Serial.print(" len=");
    Serial.print(lastHuskyDataLength);
    Serial.print(" err=");
    Serial.println(lastHuskyError);
    handleDataJson();
}

void handleStart() {
    startCapture();
    server.send(200, "text/plain", "OK");
}

void handleStop() {
    stopAndAnalyze();
    server.send(200, "text/plain", "OK");
}

void handleOtaDone() {
    server.sendHeader("Connection", "close");
    server.send(Update.hasError() ? 500 : 200, "text/plain", Update.hasError() ? "FAIL" : "OK, restarting");
    if (!Update.hasError()) {
        otaRestartPending = true;
        otaRestartAtMs = millis() + 800;
    }
}

void handleOtaUpload() {
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        capturing = false;
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("OTA Success: %u bytes\n", upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    }
}

void handleCaptivePortal() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
}

void startWebServer() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(kApSsid, kApPassword);
    connectSta();
    dnsServer.start(53, "*", WiFi.softAPIP());
    server.on("/", HTTP_GET, handleRoot);
    server.on("/generate_204", HTTP_GET, handleCaptivePortal);
    server.on("/gen_204", HTTP_GET, handleCaptivePortal);
    server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
    server.on("/library/test/success.html", HTTP_GET, handleRoot);
    server.on("/ncsi.txt", HTTP_GET, handleCaptivePortal);
    server.on("/connecttest.txt", HTTP_GET, handleCaptivePortal);
    server.on("/data.json", HTTP_GET, handleDataJson);
    server.on("/samples.csv", HTTP_GET, handleSamplesCsv);
    server.on("/config", HTTP_POST, handleConfig);
    server.on("/test-husky", HTTP_POST, handleHuskyTest);
    server.on("/start", HTTP_POST, handleStart);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/ota", HTTP_POST, handleOtaDone, handleOtaUpload);
    server.onNotFound(handleCaptivePortal);
    server.begin();
}

void showResult(const AnalysisResult &result) {
    clearScreen();
    if (!result.ok) {
        drawTextLine("Analyze failed", 8, 0xFF0000);
        drawTextLine("Need clear motion", 34, 0xFFFF00);
        drawTextLine(String("samples=") + String(result.valid), 60, 0xFFFF00);
        drawTextLine("A: retry", 96);
        k10.canvas->updateCanvas();
        return;
    }

    drawTextLine("Result", 6, 0x00FF00);
    drawTextLine("2D ellipse fit", 30, 0x00FFFF);
    drawTextLine(String("f=") + String(result.frequencyHz, 3) + " Hz", 58, 0xFFFF00);
    drawTextLine(String("T=") + String(result.periodS, 3) + " s", 82, 0xFFFF00);
    drawTextLine(String("A=") + String(result.amplitudePx, 1) + "/" + String(result.minorAmplitudePx, 1), 106, 0xFFFF00);
    drawTextLine(String("rmse=") + String(result.fitRmsePx, 1) + " px", 130, 0xFFFF00);
    drawTextLine(String("g=") + String(result.g, 3) + " m/s2", 154, 0xFFFF00);
    drawTextLine(String("fps=") + String(result.captureFps, 1), 178, 0x00FF00);
    drawTextLine("A: new capture", 212, 0x00FF00);
    k10.canvas->updateCanvas();

    Serial.println("analysis=ok");
    Serial.print("frequency_hz=");
    Serial.println(result.frequencyHz, 6);
    Serial.print("capture_fps=");
    Serial.println(result.captureFps, 3);
    Serial.print("fit_rmse_px=");
    Serial.println(result.fitRmsePx, 3);
    Serial.print("g_m_s2=");
    Serial.println(result.g, 6);
}

void startCapture() {
    sampleCount = 0;
    lostFrames = 0;
    lastSeenBlockCount = 0;
    lastMatchedId = 0;
    lastBlockWidth = 0;
    lastBlockHeight = 0;
    lastAnalysis = AnalysisResult();
    captureStartMs = millis();
    capturing = true;
    clearScreen();
    drawHud();
    Serial.println("capture=start");
}

void stopAndAnalyze() {
    capturing = false;
    Serial.println("capture=stop");
    lastAnalysis = analyzeSamples();
    showResult(lastAnalysis);
}

void pollHuskyLens() {
    if (!capturing || sampleCount >= kMaxSamples) {
        return;
    }

    float x = 0.0f;
    float y = 0.0f;
    if (readBobCenter(x, y)) {
        lastX = x;
        lastY = y;
        lastTargetMs = millis();
        lostFrames = 0;
        sampleT[sampleCount] = (millis() - captureStartMs) / 1000.0f;
        sampleX[sampleCount] = x;
        sampleY[sampleCount] = y;
        sampleCount++;
    } else {
        lostFrames++;
    }

    if (sampleCount >= kMaxSamples) {
        stopAndAnalyze();
    }
    if (sampleCount >= kMinValidSamples && millis() - captureStartMs > 60000) {
        stopAndAnalyze();
    }
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(500);

    k10.begin();
    k10.initScreen(kScreenDir);
    k10.creatCanvas();
    clearScreen();
    loadSettings();

    Wire.begin();
    Wire.setClock(100000);
    huskyReady = initHuskyLens();
    startWebServer();

    k10.buttonA->setPressedCallback(startCapture);
    k10.buttonB->setPressedCallback(stopAndAnalyze);

    Serial.println("K10 HUSKYLENS color pendulum analyzer ready.");
    Serial.println(huskyReady ? "huskylens_i2c=ok" : "huskylens_i2c=failed");
    Serial.print("huskylens_i2c_address=0x");
    Serial.println(kHuskyI2cAddress, HEX);
    Serial.print("target_id=");
    Serial.println(kTargetId);
    Serial.print("pendulum_length_m=");
    Serial.println(pendulumLengthM, 3);
    Serial.print("web_ap_ssid=");
    Serial.println(kApSsid);
    Serial.print("web_ap_url=http://");
    Serial.println(WiFi.softAPIP());
    Serial.print("wifi_sta_ssid=");
    Serial.println(staSsid);
    drawHud();
}

void loop() {
    dnsServer.processNextRequest();
    server.handleClient();
    if (otaRestartPending && int32_t(millis() - otaRestartAtMs) >= 0) {
        ESP.restart();
    }

    if (!huskyReady && millis() - lastPollMs >= 1000) {
        lastPollMs = millis();
        huskyReady = initHuskyLens();
        drawHud();
    }

    if (capturing && huskyReady && millis() - lastPollMs >= kHuskyPollIntervalMs) {
        lastPollMs = millis();
        pollHuskyLens();
    }

    if (capturing && millis() - lastHudMs >= kHudIntervalMs) {
        lastHudMs = millis();
        drawHud();
    }
    delay(1);
}