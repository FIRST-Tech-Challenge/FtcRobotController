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

public class Hardware {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx leftSlider;
    public DcMotorEx rightSlider;

    public DcMotorEx leftArm;
    public DcMotorEx rightArm;

    public DcMotorEx[] motors;
    public DcMotor fd;
    public IMU imu;

    public VoltageSensor batteryVoltageSens;


    public Servo droneServo;

    public Hardware(HardwareMap hardwareMap) {
        //initialize variables
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft.getCurrentPosition();

        leftArm = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightArm = hardwareMap.get(DcMotorEx.class, "rightArm");

        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");

        droneServo = hardwareMap.get(Servo.class, "droneLauncher");



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
