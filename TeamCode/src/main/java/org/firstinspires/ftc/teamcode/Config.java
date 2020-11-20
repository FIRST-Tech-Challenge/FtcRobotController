package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

public class Config {
    HardwareMap hmap = null;

    public DcMotor leftMotorBack = null;
    public DcMotor leftMotorFront = null;
    public DcMotor rightMotorBack = null;
    public DcMotor rightMotorFront = null;
    public DcMotor armMotor = null;

    public Servo servoArm = null;
    public Servo servoClaw = null;

    public ModernRoboticsI2cColorSensor colorSensor = null;

    public void init(HardwareMap h) {
        hmap = h;

        leftMotorBack = hmap.get(DcMotor.class, "left_motor_back");
        leftMotorFront = hmap.get(DcMotor.class, "left_motor_front");
        rightMotorBack = hmap.get(DcMotor.class, "right_motor_back");
        rightMotorFront = hmap.get(DcMotor.class, "right_motor_front");
        armMotor = hmap.get(DcMotor.class, "arm_motor");

        servoArm = hmap.get(Servo.class, "servo_arm");
        servoClaw = hmap.get(Servo.class, "servo_claw");


        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);

        // De aici in jos puteti sa ignorati ce scrie dar e pentru good practise
        // Motor run mode using the encoders
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
