package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface DriveTrain {
    void init();

    void reset();

    void setDirection(DcMotor motor, DcMotor.Direction direction);

    void setZeroPowerBehavior(DcMotor motor, DcMotor.ZeroPowerBehavior powerBehavior);

    void setMode(DcMotor motor, DcMotor.RunMode runMode);

    void stopMotor(DcMotor motor);

    void setPower(DcMotor motor, double power);

    void setTargetPosition(DcMotor motor, int targetPosition);

    void driveByTime(double power, long time);

    void driveStraight(double power, long time);

    void drive(double speed, double leftInches, double rightInches, double timeout);

    void turnRight(int degrees);

    void turnLeft(int degrees);

}
