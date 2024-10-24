package org.firstinspires.ftc.teamcode.Usefuls.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
Interface for DcMotor and Servo
*/

public interface Motor {
    public Motor setLowerBound(double bound);
    public Motor setUpperBound(double bound);
    public Motor setDirection(DcMotor.Direction direction);
    public Motor setPosition(double position);
    public Motor addPosition(double position);
    public void resetEncoder();
    public void update();
}
