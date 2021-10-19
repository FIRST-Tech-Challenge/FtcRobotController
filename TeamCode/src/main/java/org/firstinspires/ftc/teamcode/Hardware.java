package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Hardware{
    /* Public OpMode members. */
    public Motor m0 = null;
    public Motor m1 = null;
    public Motor m2 = null;
    public Motor m3 = null;

    public double MIN_ANGLE;
    public double MAX_ANGLE;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    MecanumDrive mecanum;
    DifferentialDrive m_drive;

    DistanceSensor distance;
//    ServoEx servo;

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        m0 = new Motor(hwMap, "m0");
        m1 = new Motor(hwMap, "m1");
        m2 = new Motor(hwMap, "m2");
        m3 = new Motor(hwMap, "m3");

        m0.set(0);
        m1.set(0);
        m2.set(0);
        m3.set(0);

        m0.setInverted(false);
        m1.setInverted(true);
        m2.setInverted(false);
        m3.setInverted(true);

        m0.setRunMode(Motor.RunMode.VelocityControl);
        m1.setRunMode(Motor.RunMode.VelocityControl);
        m2.setRunMode(Motor.RunMode.VelocityControl);
        m3.setRunMode(Motor.RunMode.VelocityControl);

        m0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mecanum = new MecanumDrive(m0, m2, m1, m3);

        MotorGroup m_left = new MotorGroup(m0, m2);
        MotorGroup m_right = new MotorGroup(m1, m3);

        m_drive = new DifferentialDrive(m_left, m_right);
        distance = hwMap.get(DistanceSensor.class, "Distance");

//        servo = new SimpleServo(hwMap, "servo", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
    }
 }

