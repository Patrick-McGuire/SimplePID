#include <SimplePID.h>

SimplePID pid(5, .1, 0);

void setup() {
    // Setup Serial
    Serial.begin(9600);
    Serial.println();
    Serial.println("PID Tester");

    Serial.print("KP: ");
    Serial.println(pid.getKp());
    Serial.print("KI: ");
    Serial.println(pid.getKi());
    Serial.print("KD: ");
    Serial.println(pid.getKd());

    pid.updateSetpoint(5.5);
    pid.setIntegratorLimit(10.5);
    pid.setOutputLimit(20);
}

void loop() {
    double state = getFloat();
    Serial.println(pid.compute(state));
}

float getFloat() {
    while(!Serial.available()) {
        ;
    }
    delay(50);
    return Serial.parseFloat();
}