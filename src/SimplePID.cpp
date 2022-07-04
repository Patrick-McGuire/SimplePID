#include "Arduino.h"
#include "SimplePID.h"

SimplePID::SimplePID(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void SimplePID::updateSetpoint(double _setpoint) {
    setpoint = _setpoint;
}

double SimplePID::compute(double state, double _setpoint) {
    updateSetpoint(_setpoint);
    return compute(state);
}

double SimplePID::compute(double state) {
    double p, i = 0, d = 0;
    // Calculate the error
    double error = setpoint - state;
    // Calculate change in time
    double time = double(millis()) / 1000;
    double dt = time - lastTime;
    lastTime = time;
    // Calculate the components
    p = error * kp;
    if(initialized) {
        integrator += error * dt;
        integrator = integratorLim == 0 ? integrator : constrain(integrator, -integratorLim / ki, integratorLim / ki);
        i = integrator * ki;
        d = ((error - lastError) / dt) * kd;
    }
    // Reset var
    initialized = true;
    lastError = error;
    // Calculate the output
    return outputLim == 0 ? p + i + d : constrain(p + i + d, -outputLim, outputLim);
}


//// Getter and setter
void SimplePID::setOutputLimit(double lim) {
    outputLim = abs(lim);
}

void SimplePID::setIntegratorLimit(double lim) {
    integratorLim = abs(lim);
}

double SimplePID::getKp() const {
    return kp;
}

void SimplePID::setKp(double kp) {
    SimplePID::kp = kp;
}

double SimplePID::getKi() const {
    return ki;
}

void SimplePID::setKi(double ki) {
    SimplePID::ki = ki;
}

double SimplePID::getKd() const {
    return kd;
}

void SimplePID::setKd(double kd) {
    SimplePID::kd = kd;
}

