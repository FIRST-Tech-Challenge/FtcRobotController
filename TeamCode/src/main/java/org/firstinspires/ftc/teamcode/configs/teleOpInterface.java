package org.firstinspires.ftc.teamcode.configs;

import java.util.HashMap;

public interface teleOpInterface {
    HashMap<String, String> telemetryDM = new HashMap<>();

    //Four buttons
    void a(boolean pressed);
    void b(boolean pressed);
    void x(boolean pressed);
    void y(boolean pressed);

    //D-pad
    void dd(boolean pressed);
    void dp(boolean pressed);
    void dl(boolean pressed);
    void dr(boolean pressed);

    //Right side bumper and trigger
    void rb(boolean pressed);
    void rt(float pressure);

    //Left side bumper and trigger
    void lb(boolean pressed);
    void lt(float pressure);

    //right and left joystick and button
    void rjoy(float x, float y);
    void ljoy(float x, float y);
    void rjoyb(boolean pressed);
    void ljoyb(boolean pressed);

    void custom1();

    void updateTelemetryDM();

    void loop();

    String getName();

}
