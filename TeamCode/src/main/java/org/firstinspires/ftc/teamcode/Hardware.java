package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.AnalogCommands;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Hardware {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx armMotor1;
    public DcMotorEx armMotor2;
    public DcMotorEx leftSlider;
    public DcMotorEx rightSlider;
    public DcMotorEx[] motors;
    public IMU imu;

    public VoltageSensor batteryVoltageSens;

//servos

    public Servo droneLauncher;
    public Servo clawFront;
    public Servo clawBack;
    public Servo clawWrist;

    public Hardware(HardwareMap hardwareMap) {
        //initialize variables
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft.getCurrentPosition();

        armMotor1 = hardwareMap.get(DcMotorEx.class, "leftArm");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "rightArm");

        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");
        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");

        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        clawFront = hardwareMap.get(Servo.class, "clawFrontServo");
        clawBack = hardwareMap.get(Servo.class, "clawBackServo");
        clawWrist = hardwareMap.get(Servo.class, "ClawMountServo");



        motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Control Hub

        // Battery
        batteryVoltageSens = hardwareMap.voltageSensor.iterator().next();

        //


        // Fix the things

        if (RUN_USING_ENCODER) { // We are running using encoders
            for (DcMotor motor : motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}

