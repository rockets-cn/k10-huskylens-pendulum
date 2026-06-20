#include "unihiker_k10.h"

#include <Wire.h>
#include <math.h>

UNIHIKER_K10 k10;

namespace {
// ===== Experiment settings to edit before upload =====
constexpr float kPendulumLengthM = 0.420f;  // pivot to bob center, meters
constexpr int kPivotX = 160;                // pivot pixel x in the HUSKYLENS image
constexpr int kPivotY = 24;                 // pivot pixel y in the HUSKYLENS image
constexpr uint16_t kTargetId = 1;           // learned HUSKYLENS object ID for the bob

// ===== Capture settings =====
constexpr uint8_t kScreenDir = 2;
constexpr uint16_t kMaxSamples = 512;
constexpr uint16_t kMinValidSamples = 24;
constexpr uint32_t kHudIntervalMs = 250;
constexpr uint32_t kHuskyPollIntervalMs = 20;
constexpr uint32_t kHuskyReadTimeoutMs = 80;

// ===== HUSKYLENS protocol =====
constexpr uint8_t kHuskyI2cAddress = 0x32;
constexpr uint8_t kHuskyProtocolAddress = 0x11;
constexpr uint8_t kCommandRequestBlocksById = 0x27;
constexpr uint8_t kCommandRequestKnock = 0x2C;
constexpr uint8_t kCommandRequestAlgorithm = 0x2D;
constexpr uint8_t kCommandReturnInfo = 0x29;
constexpr uint8_t kCommandReturnBlock = 0x2A;
constexpr uint8_t kCommandReturnOk = 0x2E;
constexpr uint8_t kAlgorithmObjectTracking = 0x01;
constexpr uint8_t kMaxHuskyFrameBytes = 16;

struct AnalysisResult {
    bool ok = false;
    uint16_t valid = 0;
    float x0 = 0.0f;
    float amplitudePx = 0.0f;
    float thetaRad = 0.0f;
    float frequencyHz = 0.0f;
    float periodS = 0.0f;
    float g = 0.0f;
    float captureFps = 0.0f;
};

uint32_t lastHudMs = 0;
uint32_t lastPollMs = 0;
uint32_t lastTargetMs = 0;
uint16_t sampleCount = 0;
bool capturing = false;
bool huskyReady = false;
uint16_t lostFrames = 0;
float lastX = 0.0f;
float lastY = 0.0f;

float sampleT[kMaxSamples];
float sampleX[kMaxSamples];
float sampleY[kMaxSamples];

void drawTextLine(const String &text, int16_t y, uint32_t color = 0x00FF00) {
    k10.canvas->canvasText(text, 4, y, color, k10.canvas->eCNAndENFont16, 150, true);
}

void clearScreen() {
    k10.setScreenBackground(0x000000);
    k10.canvas->canvasClear();
}

void drawHud() {
    drawTextLine("HUSKY pendulum", 6);
    drawTextLine(capturing ? "A:capturing  B:result" : "A:start  B:result", 30);
    drawTextLine(String("samples: ") + String(sampleCount), 54);
    drawTextLine(huskyReady ? "I2C:ok" : "I2C:check", 78, huskyReady ? 0x00FF00 : 0xFF0000);
    if (lastTargetMs > 0) {
        drawTextLine(String("x,y=") + String(lastX, 0) + "," + String(lastY, 0), 102, 0x00FFFF);
        drawTextLine(String("lost: ") + String(lostFrames), 126, lostFrames == 0 ? 0x00FF00 : 0xFFFF00);
    }
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

bool sendHuskyCommand(uint8_t command, const uint8_t *data = nullptr, uint8_t dataLength = 0) {
    uint8_t frame[kMaxHuskyFrameBytes] = {0x55, 0xAA, kHuskyProtocolAddress, dataLength, command};
    for (uint8_t i = 0; i < dataLength; ++i) {
        frame[5 + i] = data[i];
    }
    const uint8_t frameLength = 5 + dataLength + 1;
    frame[frameLength - 1] = checksum(frame, frameLength - 1);

    Wire.beginTransmission(kHuskyI2cAddress);
    Wire.write(frame, frameLength);
    return Wire.endTransmission() == 0;
}

bool readHuskyFrame(uint8_t &command, uint8_t *data, uint8_t &dataLength, uint32_t timeoutMs = kHuskyReadTimeoutMs) {
    const uint32_t deadline = millis() + timeoutMs;
    while (int32_t(deadline - millis()) > 0) {
        uint8_t raw[kMaxHuskyFrameBytes] = {};
        uint8_t count = 0;
        Wire.requestFrom(kHuskyI2cAddress, kMaxHuskyFrameBytes);
        while (Wire.available() && count < kMaxHuskyFrameBytes) {
            raw[count++] = Wire.read();
        }

        for (uint8_t offset = 0; offset + 5 < count; ++offset) {
            if (raw[offset] != 0x55 || raw[offset + 1] != 0xAA || raw[offset + 2] != kHuskyProtocolAddress) {
                continue;
            }
            const uint8_t len = raw[offset + 3];
            const uint8_t frameLength = 5 + len + 1;
            if (len > 10 || offset + frameLength > count) {
                continue;
            }
            if (checksum(&raw[offset], frameLength - 1) != raw[offset + frameLength - 1]) {
                continue;
            }

            command = raw[offset + 4];
            dataLength = len;
            for (uint8_t i = 0; i < len; ++i) {
                data[i] = raw[offset + 5 + i];
            }
            return true;
        }
        delay(2);
    }
    return false;
}

bool readOkFrame(uint32_t timeoutMs = kHuskyReadTimeoutMs) {
    uint8_t command = 0;
    uint8_t data[10] = {};
    uint8_t dataLength = 0;
    return readHuskyFrame(command, data, dataLength, timeoutMs) && command == kCommandReturnOk;
}

bool initHuskyLens() {
    if (!sendHuskyCommand(kCommandRequestKnock) || !readOkFrame()) {
        return false;
    }

    const uint8_t algorithmData[2] = {kAlgorithmObjectTracking, 0x00};
    if (!sendHuskyCommand(kCommandRequestAlgorithm, algorithmData, sizeof(algorithmData))) {
        return false;
    }
    return readOkFrame(250);
}

bool readBobCenter(float &cx, float &cy) {
    const uint8_t idData[2] = {uint8_t(kTargetId & 0xFF), uint8_t(kTargetId >> 8)};
    if (!sendHuskyCommand(kCommandRequestBlocksById, idData, sizeof(idData))) {
        return false;
    }

    uint8_t command = 0;
    uint8_t data[10] = {};
    uint8_t dataLength = 0;
    if (!readHuskyFrame(command, data, dataLength) || command != kCommandReturnInfo || dataLength < 2) {
        return false;
    }

    const uint16_t blockCount = readLe16(&data[0]);
    for (uint16_t i = 0; i < blockCount; ++i) {
        if (!readHuskyFrame(command, data, dataLength) || command != kCommandReturnBlock || dataLength < 10) {
            return false;
        }

        const uint16_t id = readLe16(&data[8]);
        if (id == kTargetId) {
            cx = readLe16(&data[0]);
            cy = readLe16(&data[2]);
            return true;
        }
    }
    return false;
}

float estimateFrequency(float meanX) {
    float crossings[64];
    uint8_t crossingCount = 0;

    for (uint16_t i = 1; i < sampleCount; ++i) {
        const float y0 = sampleX[i - 1] - meanX;
        const float y1 = sampleX[i] - meanX;
        if (y0 < 0.0f && y1 >= 0.0f && crossingCount < 64) {
            const float frac = -y0 / (y1 - y0 + 1e-6f);
            crossings[crossingCount++] = sampleT[i - 1] + frac * (sampleT[i] - sampleT[i - 1]);
        }
    }

    if (crossingCount < 2) {
        return 0.0f;
    }

    float totalPeriod = 0.0f;
    for (uint8_t i = 1; i < crossingCount; ++i) {
        totalPeriod += crossings[i] - crossings[i - 1];
    }
    const float period = totalPeriod / (crossingCount - 1);
    return period > 0.0f ? 1.0f / period : 0.0f;
}

AnalysisResult analyzeSamples() {
    AnalysisResult result;
    result.valid = sampleCount;
    if (sampleCount < kMinValidSamples) {
        return result;
    }

    float sumX = 0.0f;
    float minX = 100000.0f;
    float maxX = -100000.0f;
    float sumLengthPx = 0.0f;

    for (uint16_t i = 0; i < sampleCount; ++i) {
        const float x = sampleX[i];
        const float y = sampleY[i];
        sumX += x;
        minX = min(minX, x);
        maxX = max(maxX, x);
        const float dx = x - kPivotX;
        const float dy = y - kPivotY;
        sumLengthPx += sqrtf(dx * dx + dy * dy);
    }

    result.x0 = sumX / sampleCount;
    result.amplitudePx = (maxX - minX) * 0.5f;
    result.frequencyHz = estimateFrequency(result.x0);
    if (result.frequencyHz <= 0.0f) {
        return result;
    }

    const float captureTime = sampleT[sampleCount - 1] - sampleT[0];
    result.captureFps = captureTime > 0.0f ? (sampleCount - 1) / captureTime : 0.0f;

    const float lengthPx = sumLengthPx / sampleCount;
    result.thetaRad = asinf(min(0.999f, result.amplitudePx / max(1.0f, lengthPx)));
    result.periodS = 1.0f / result.frequencyHz;
    const float omega = 2.0f * PI * result.frequencyHz;
    result.g = omega * omega * kPendulumLengthM;
    result.ok = true;
    return result;
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
    drawTextLine("x=x0+Acos(2pi*f*t)", 30, 0x00FFFF);
    drawTextLine(String("f=") + String(result.frequencyHz, 3) + " Hz", 58, 0xFFFF00);
    drawTextLine(String("T=") + String(result.periodS, 3) + " s", 82, 0xFFFF00);
    drawTextLine(String("A=") + String(result.amplitudePx, 1) + " px", 106, 0xFFFF00);
    drawTextLine(String("theta=") + String(result.thetaRad, 3), 130, 0xFFFF00);
    drawTextLine(String("g=") + String(result.g, 3) + " m/s2", 154, 0xFFFF00);
    drawTextLine(String("fps=") + String(result.captureFps, 1), 178, 0x00FF00);
    drawTextLine("A: new capture", 212, 0x00FF00);
    k10.canvas->updateCanvas();

    Serial.println("analysis=ok");
    Serial.print("frequency_hz=");
    Serial.println(result.frequencyHz, 6);
    Serial.print("capture_fps=");
    Serial.println(result.captureFps, 3);
    Serial.print("g_m_s2=");
    Serial.println(result.g, 6);
}

void startCapture() {
    sampleCount = 0;
    lostFrames = 0;
    capturing = true;
    clearScreen();
    drawHud();
    Serial.println("capture=start");
}

void stopAndAnalyze() {
    capturing = false;
    Serial.println("capture=stop");
    showResult(analyzeSamples());
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
        sampleT[sampleCount] = millis() / 1000.0f;
        sampleX[sampleCount] = x;
        sampleY[sampleCount] = y;
        sampleCount++;
    } else {
        lostFrames++;
    }

    if (sampleCount >= kMaxSamples) {
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

    Wire.begin();
    Wire.setClock(100000);
    huskyReady = initHuskyLens();

    k10.buttonA->setPressedCallback(startCapture);
    k10.buttonB->setPressedCallback(stopAndAnalyze);

    Serial.println("K10 HUSKYLENS pendulum analyzer ready.");
    Serial.println(huskyReady ? "huskylens_i2c=ok" : "huskylens_i2c=failed");
    Serial.print("huskylens_i2c_address=0x");
    Serial.println(kHuskyI2cAddress, HEX);
    Serial.print("target_id=");
    Serial.println(kTargetId);
    drawHud();
}

void loop() {
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
