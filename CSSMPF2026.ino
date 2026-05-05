extern "C" {
    #include "estimator.h" // pulls in pid_motor_mixer.h and imu_types.h already
}

#include <string.h> // memset


// Gains — tune these experimentally
gain roll_gains  = {50.0f, 0.0f, 5.0f};
gain pitch_gains = {50.0f, 0.0f, 5.0f};
gain yaw_gains   = {30.0f, 0.0f, 20.0f};

state roll_state  = {0};
state pitch_state = {0};
state yaw_state   = {0};

estimator est       = {0};
pid       pid_out   = {0};
output    motor_out = {0};

// Configure the estimator
static void configure_estimator() {
    Estimator_Config cfg;
    cfg.ax_bias = 0.0f; cfg.ay_bias = 0.0f; cfg.az_bias = 0.0f;
    cfg.gx_bias = 0.0f; cfg.gy_bias = 0.0f; cfg.gz_bias = 0.0f;
    cfg.alpha   = 0.98f;     // ~50 ms gyro time constant at 100 Hz
    Estimator_SetConfig(&cfg);
}

// Returns true once the three header bytes have been consumed in order.
static bool read_header_synced() {
    static char hdr[3] = {0};
    while (Serial.available() > 0) {
        hdr[0] = hdr[1];
        hdr[1] = hdr[2];
        hdr[2] = (char)Serial.read();
        if (hdr[0] == 'I' && hdr[1] == 'M' && hdr[2] == 'U') {
            hdr[0] = hdr[1] = hdr[2] = 0;   // reset for next packet
            return true;
        }
    }
    return false;
}

// Session tracking. If we go silent for longer than this, the
// next packet is treated as the start of a fresh session and we re-init.
static unsigned long last_packet_ms = 0;
static const unsigned long RESET_GAP_MS = 200;

// Re-initialise everything that holds state across loop iterations.
// Called from setup() and again whenever a fresh session is detected.
static void reset_runtime_state() {
    Estimator_Init();           // wipes prev_roll/pitch/yaw and config
    configure_estimator();      // re-applies the desired alpha
    memset(&roll_state,  0, sizeof(roll_state));
    memset(&pitch_state, 0, sizeof(pitch_state));
    memset(&yaw_state,   0, sizeof(yaw_state));
    memset(&est,         0, sizeof(est));
    memset(&pid_out,     0, sizeof(pid_out));
    memset(&motor_out,   0, sizeof(motor_out));
}

void setup() {
    Serial.begin(115200);
    reset_runtime_state();
}

void loop() {
    // Wait until at least the header has arrived
    if (Serial.available() < 3) return;

    if (!read_header_synced()) return;

    // After the header is found, wait for the 40-byte payload
    while (Serial.available() < 40) { /* spin briefly */ }

    // Read 10 floats (40 bytes)
    // buf: [gx, gy, gz, ax, ay, az, target_roll, target_pitch, target_yaw, dt]
    float buf[10];
    Serial.readBytes((char*)buf, 40);

    // If the host has been silent for longer than RESET_GAP_MS, treat this as the first packet of a new
    // session and wipe all carryover state. This is the workaround for
    // boards that don't auto-reset on serial open — without it, a tumbled
    // run leaves enormous values in the estimator's prev_* statics, and the
    // next "fresh" run starts on top of that garbage.
    unsigned long now = millis();
    if (now - last_packet_ms > RESET_GAP_MS) {
        reset_runtime_state();
    }
    last_packet_ms = now;

    // Unpack into IMU struct
    IMU_Data imu;
    imu.gx = buf[0]; imu.gy = buf[1]; imu.gz = buf[2];
    imu.ax = buf[3]; imu.ay = buf[4]; imu.az = buf[5];

    est.roll_setpoint  = buf[6];
    est.pitch_setpoint = buf[7];
    est.yaw_setpoint   = buf[8];
    est.dt             = buf[9];

    // Estimate current attitude from IMU
    Estimator_Update(&imu, &est);

    // Run PID
    roll_PID (&roll_gains,  &est, &pid_out, &roll_state);
    pitch_PID(&pitch_gains, &est, &pid_out, &pitch_state);
    yaw_PID  (&yaw_gains,   &est, &pid_out, &yaw_state);

    // Acceleration PIDs intentionally omitted

    // Mix PID outputs into per-motor commands
    motor_mixer(&motor_out, &est, &pid_out);

    // Send back 4 motor values (16 bytes)
    float motors[4] = {
        motor_out.spinMotorOne,
        motor_out.spinMotorTwo,
        motor_out.spinMotorThree,
        motor_out.spinMotorFour
    };
    Serial.write((char*)motors, 16);
}