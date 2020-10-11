package org.firstinspires.ftc.teamcode;

public interface teleOpInterface {

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
    void rt();

    //Left side bumper and trigger
    void lb();
    void lt();

    //right and left joystick and button
    void rjoy(float x, float y);
    void ljoy(float x, float y);
    void rjoyb();
    void ljoyb();

    void custom1();
}
