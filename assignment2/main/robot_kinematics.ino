const float WHEEL_RADIUS = 3.2;
const float HALF_WHEELBASE = 6.85;   // b

const float M_F = 0.0381;
const float C_F = 1.91;

// Forward: wheel speeds (rad/s) -> v (cm/s), w (rad/s)
void wheelsToVelocity(float phiRight, float phiLeft, float &v, float &w) {
    v = WHEEL_RADIUS / 2.0 * (phiRight + phiLeft);
    w = WHEEL_RADIUS / (2.0 * HALF_WHEELBASE) * (phiRight - phiLeft);
}

// Inverse: v (cm/s), w (rad/s) -> wheel speeds (rad/s)
void velocityToWheels(float v, float w, float &phiRight, float &phiLeft) {
    phiRight = (v + HALF_WHEELBASE * w) / WHEEL_RADIUS;
    phiLeft  = (v - HALF_WHEELBASE * w) / WHEEL_RADIUS;
}

// PWM -> wheel speed (rad/s)
float pwmToWheelSpeed(int pwm) {
    return M_F * pwm + C_F;
}

// Wheel speed (rad/s) -> PWM (0 to 255)
int wheelSpeedToPwm(float phi) {
    int pwm = (phi - C_F) / M_F;
    return constrain(pwm, 0, 255);
}