#ifndef SIMPLEPID_SIMPLEPID_H
#define SIMPLEPID_SIMPLEPID_H

#include "Arduino.h"

class SimplePID {
public:
    /**
     * Constructor
     * @param _kp proportional constant
     * @param _ki integrator constant
     * @param _kd derivative constant
     */
    explicit SimplePID(double _kp = 0, double _ki = 0, double _kd = 0);

    /**
     * Sets the max/min values compute() can return
     * @param lim -min/max value
     */
    void setOutputLimit(double lim);

    /**
     * Sets the max/min values the integrator can accrue can return
     * @param lim -min/max value
     */
    void setIntegratorLimit(double lim);

    /**
     * Sets the current setpoint
     * @param _setpoint target value
     */
    void updateSetpoint(double _setpoint);

    /**
     * Computes the PID algorithm
     * @param state current state
     * @param _setpoint desired state
     * @return PID output
     */
    double compute(double state, double _setpoint);

    /**
     * Computes the PID algorithm
     * @param state current state
     * @return PID output
     */
    double compute(double state);

    //// Basic getter/setter

    double getKp() const;

    void setKp(double kp);

    double getKi() const;

    void setKi(double ki);

    double getKd() const;

    void setKd(double kd);

private:
    double kp;
    double ki;
    double kd;

    double integratorLim = 0;
    double outputLim = 0;

    double setpoint = 0;
    double integrator = 0;
    double lastError = 0;
    double lastTime = 0;
    bool initialized = false;
};


#endif //SIMPLEPID_SIMPLEPID_H
