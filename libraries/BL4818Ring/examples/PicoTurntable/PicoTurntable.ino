#include <BL4818Ring.h>

// Example settings only: tune for the actual reduction and load.
constexpr uint8_t MOTOR = 0;
constexpr int POT_PIN = 26;
constexpr int POT_STOP = 80;
constexpr int MAX_MOTOR_RPM = 100;  // Table RPM = motor RPM / reduction ratio.
constexpr int RPM_STEP = 1;        // At most 1 motor RPM every 20 ms.
constexpr uint16_t TORQUE_LIMIT_MA = 500;
BL4818Ring ring(Serial1);
bool ready = false;
bool zero_seen = false;
int filtered_pot = 0;
int commanded_rpm = 0;
uint32_t last_step = 0;
uint32_t last_status = 0;

void latch_error()
{
    ready = false;
    // Best effort only: a broken link cannot deliver STOP.
    ring.stop(MOTOR);
    Serial.println("Stopped controller task; check motor/link, then reset Pico.");
}

void setup()
{
    Serial.begin(115200);  // USB diagnostics; do not wait for a USB host.
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.setFIFOSize(256);
    Serial1.begin(BL4818Ring::BAUD);
    analogReadResolution(12);
    filtered_pot = analogRead(POT_PIN);
    delay(1000);  // Allow the motor application to finish booting.
    uint8_t count = 0;
    if (!ring.enumerate(count))
    {
        Serial.println("Enumeration failed; no motion commands sent.");
        return;
    }
    // This example owns a dedicated one-motor ring.
    if (count != 1)
    {
        Serial.println("Expected exactly one motor; no motion commands sent.");
        return;
    }
    BL4818Ring::Status status;
    if (!ring.stop(MOTOR) || !ring.query_status(MOTOR, status) || status.fault ||
        !ring.set_torque_limit(MOTOR, TORQUE_LIMIT_MA))
    {
        latch_error();
        return;
    }
    ready = true;
    Serial.println("Turn knob to zero to arm.");
}

void loop()
{
    if (!ready) return;
    uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_step) < 20) return;
    last_step = now;
    int raw = analogRead(POT_PIN);
    filtered_pot += (raw - filtered_pot) / 8;
    if (!zero_seen)
    {
        zero_seen = raw <= POT_STOP && filtered_pot <= POT_STOP;
        return;
    }
    int target = filtered_pot <= POT_STOP ? 0 :
        static_cast<long>(filtered_pot - POT_STOP) * MAX_MOTOR_RPM / (4095 - POT_STOP);
    int next = commanded_rpm;
    if (next < target) next += min(RPM_STEP, target - next);
    if (next > target) next -= min(RPM_STEP, next - target);
    if (next != commanded_rpm)
    {
        // Velocity zero is an active control mode; use STOP to disable drive.
        bool ok = next == 0 ? ring.stop(MOTOR) : ring.set_velocity(MOTOR, next);
        if (!ok) { latch_error(); return; }
        commanded_rpm = next;
    }
    if (static_cast<uint32_t>(now - last_status) >= 250)
    {
        last_status = now;
        BL4818Ring::Status status;
        if (!ring.query_status(MOTOR, status) || status.fault ||
            (commanded_rpm != 0 && (status.state != 1 || status.mode != 1 ||
                                    status.target_rpm != commanded_rpm)))
        {
            latch_error();
            return;
        }
    }
}
