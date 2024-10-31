package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class hardware {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor mantis;
    public DcMotor lift;
    public DcMotor hopper;

    public CRServo wrist;
    public CRServo grabber;
    public CRServo door;

    public ColorSensor colorSensor;

    public DistanceSensor distanceSensorLeft;
    public DistanceSensor distanceSensorRight;
    public DistanceSensor distanceSensorBack;

    public void checkMotorInit(){

    }
}
