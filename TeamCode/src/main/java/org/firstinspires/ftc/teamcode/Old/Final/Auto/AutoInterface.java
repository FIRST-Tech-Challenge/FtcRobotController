package org.firstinspires.ftc.teamcode.Old.Final.Auto;

import org.firstinspires.ftc.teamcode.mainEnum;

public interface AutoInterface {

    // Telemetry
    void telemetry();

    // Initialization
    void armInit();
    void wheelInit();
    void sensorInit();
    void servoInit();

    // Direction Settings
    void setArmDirection();
    void setWheelDirection();

    // Motor Braking
    void armBrake();
    void wheelBrake();

    // Speed Control
    void setWheelSpeed(mainEnum state, double speed);
    void setArmSpeed(mainEnum state, double speed);
    void setClawSpeed(mainEnum state,double speedTopGrabber, double speedBottomGrabber, double wristSpeed, int doorPos);

    // Utility Functions
    void whileMotorsBusy();
}
