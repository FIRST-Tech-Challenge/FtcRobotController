package org.firstinspires.ftc.teamcode.Auto;

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

    // Motor Position Control
    void setWheelPosition(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR);
    void wheelMotorToPosition();
    void setArmPosition(int targetPosMantis, int targetPosLift, int targetPosHopper);
    void armMotorToPosition();
    void doorPos(double position);

    // Speed Control
    void setWheelSpeed(double speedFL, double speedFR, double speedBL, double speedBR);
    void setArmSpeed(double speedMantis, double speedLift, double speedHopper);
    void setClawSpeed(double speedWrist, double speedTopGrabber, double speedBottomGrabber);

    // Utility Functions
    void whileMotorsBusy();
    void resetMotorEncoders();

    // Composite Operations
    void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR,
              double speedFL, double speedFR, double speedBL, double speedBR);
}
