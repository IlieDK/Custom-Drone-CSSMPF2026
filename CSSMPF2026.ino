// CSSMPF2026.ino
// Arduino entry point - talks to simulation.py over USB serial
// receives IMU + setpoints + dt, runs the control pipeline, sends motor commands back

extern "C" {
    #include "estimator.h" // pulls in pid_motor_mixer.h and imu_types.h
}

#include <string.h> // for memset

// PID gains - tuned for this drone
gain roll_gains  = {50.0f, 0.0f,  5.0f};  // roll  P I D
gain pitch_gains = {50.0f, 0.0f,  5.0f};  // pitch P I D
gain yaw_gains   = {30.0f, 0.0f, 20.0f};  // yaw   P I D - lower gains, yaw is a slow axis

// PID state per axis - zero-initialised
state roll_state  = {0};
state pitch_state = {0};
state yaw_state   = {0};

// shared structs that flow through the pipeline
estimator est       = {0};
pid       pid_out   = {0};
output    motor_out = {0};

// session reset stuff - see fix below
static unsigned long last_packet_ms = 0;
static const unsigned long RESET_GAP_MS = 200; // anything quieter than this counts as a fresh session


// wrapper around Estimator_SetConfig - lets us pass in our chosen alpha and zero biases
static void configure_estimator() {
    Estimator_Config cfg;
    cfg.ax_bias = 0.0f; cfg.ay_bias = 0.0f; cfg.az_bias = 0.0f; // zero biases
    cfg.gx_bias = 0.0f; cfg.gy_bias = 0.0f; cfg.gz_bias = 0.0f;
    cfg.alpha   = 0.98f; // trust gyro a lot
    Estimator_SetConfig(&cfg);
}


// slide one byte at a time looking for the 'IMU' header
// returns true once we've found it
static bool read_header_synced() {
    static char hdr[3] = {0}; // sliding window of the last 3 bytes
    while (Serial.available() > 0) {
        hdr[0] = hdr[1];
        hdr[1] = hdr[2];
        hdr[2] = (char)Serial.read();
        if (hdr[0] == 'I' && hdr[1] == 'M' && hdr[2] == 'U') {
            hdr[0] = hdr[1] = hdr[2] = 0; // reset for the next packet
            return true;
        }
    }
    return false;
}


// wipe everything that survives between calls so a fresh session starts clean
// this is needed because the Arduino doesn't always auto-reset on serial open
static void reset_runtime_state() {
    Estimator_Init();      // clear prev_roll/pitch/yaw and config
    configure_estimator(); // restore our alpha
    memset(&roll_state,  0, sizeof(roll_state));  // clear PID memory
    memset(&pitch_state, 0, sizeof(pitch_state));
    memset(&yaw_state,   0, sizeof(yaw_state));
    memset(&est,         0, sizeof(est));
    memset(&pid_out,     0, sizeof(pid_out));
    memset(&motor_out,   0, sizeof(motor_out));
}


// Arduino setup - runs once
void setup() {
    Serial.begin(115200); // open serial at the rate simulation.py uses
    reset_runtime_state(); // initialise everything
}


// Arduino main loop - runs forever
void loop() {
    // wait for at least the header to arrive
    if (Serial.available() < 3) return;

    if (!read_header_synced()) return; // try to find 'IMU'

    // got the header - now wait for the 40-byte payload (10 floats)
    while (Serial.available() < 40) { /* spin */ }

    // read the payload: gx gy gz ax ay az target_roll target_pitch target_yaw dt
    float buf[10];
    Serial.readBytes((char*)buf, 40);

    // detect a fresh session - if the host was quiet for a while, wipe state
    unsigned long now = millis();
    if (now - last_packet_ms > RESET_GAP_MS) {
        reset_runtime_state();
    }
    last_packet_ms = now;

    // unpack the IMU reading
    IMU_Data imu;
    imu.gx = buf[0]; imu.gy = buf[1]; imu.gz = buf[2];
    imu.ax = buf[3]; imu.ay = buf[4]; imu.az = buf[5];

    // setpoints and dt go on the est struct
    est.roll_setpoint  = buf[6];
    est.pitch_setpoint = buf[7];
    est.yaw_setpoint   = buf[8];
    est.dt             = buf[9];

    // step 1: estimate attitude from the IMU
    Estimator_Update(&imu, &est);

    // step 2: run the three attitude PIDs
    roll_PID (&roll_gains,  &est, &pid_out, &roll_state);
    pitch_PID(&pitch_gains, &est, &pid_out, &pitch_state);
    yaw_PID  (&yaw_gains,   &est, &pid_out, &yaw_state);
    // (acceleration PIDs intentionally left out - not driven by anything)

    // step 3: mix PIDs into per-motor commands
    motor_mixer(&motor_out, &est, &pid_out);

    // step 4: send the four motor commands back (16 bytes)
    float motors[4] = {
        motor_out.spinMotorOne,
        motor_out.spinMotorTwo,
        motor_out.spinMotorThree,
        motor_out.spinMotorFour
    };
    Serial.write((char*)motors, 16);
}