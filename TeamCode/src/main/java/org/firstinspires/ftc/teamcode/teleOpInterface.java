package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

public interface teleOpInterface {
    HashMap<String, String> telemetryDM = new HashMap<>();

    //Four buttons
    void a();
    void b();
    void x();
    void y();

    //D-pad
    void dd();
    void dp();
    void dl();
    void dr();

    //Right side bumper and trigger
    void rb();
    void rt(float pressure);

    //Left side bumper and trigger
    void lb();
    void lt(float pressure);

    //right and left joystick and button
    void rjoy(float x, float y);
    void ljoy(float x, float y);
    void rjoyb();
    void ljoyb();

    void custom1();

    void updateTelemetryDM();

}
