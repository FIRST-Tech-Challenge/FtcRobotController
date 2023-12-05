package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;


public class Hardware {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;

    public DcMotor winch = null;

    public Servo hook = null;
    public Servo gripper = null;
    public Servo arm = null;
    public Servo launcherRelease = null;
    public Servo droneAngle = null;
    public Servo droneHolder = null;

    public Rev2mDistanceSensor rightDistance = null;
    public Rev2mDistanceSensor leftDistance = null;

    HardwareMap hwMap = null;

    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        leftDrive = hwMap.get(DcMotor.class, "fl");
        rightDrive = hwMap.get(DcMotor.class, "fr");
        rightBackDrive = hwMap.get(DcMotor.class, "br");
        leftBackDrive = hwMap.get(DcMotor.class, "bl");

        winch = hwMap.get(DcMotor.class, "winch");

        hook = hwMap.get(Servo.class, "hook");
        gripper = hwMap.get(Servo.class, "grip");
        arm = hwMap.get(Servo.class, "arm");
        launcherRelease = hwMap.get(Servo.class, "release");
        droneAngle = hwMap.get(Servo.class, "angle");
        droneHolder = hwMap.get(Servo.class, "clip");

        rightDistance = hwMap.get(Rev2mDistanceSensor.class, "rdist");
        leftDistance = hwMap.get(Rev2mDistanceSensor.class, "ldist");



        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}

