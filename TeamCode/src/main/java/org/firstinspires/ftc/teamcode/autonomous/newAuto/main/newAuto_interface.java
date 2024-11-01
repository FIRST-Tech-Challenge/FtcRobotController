package org.firstinspires.ftc.teamcode.autonomous.newAuto.main;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mainEnum;

public interface newAuto_interface {
    void initialize();
    void telemetry();
    void setDirection();
    void setBrakes();
    void whileMotorsBusy();

    void runToPosition(DcMotor motor);
    void motorToPosition();

    void resetEncoder(DcMotor motor);
    void resetMotorEncoders();

    void setPosition(int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos);
    void setSpeed(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed);


    void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR, double speedFL, double speedFR, double speedBL, double speedBR);
    void movement(mainEnum state, double tick, double rotation, double speed);



    void arm(mainEnum motor, double inch, double speed);
    void grabber(mainEnum state);

    boolean detectYellow(int blue, int red, int green);

    void moveTo(mainEnum state);
}
