package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.AbstractOmniDrivetrain;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;
import java.util.Map;

public class RobotClass {
    public AbstractOmniDrivetrain drivetrain;

    public Map<MOTORS, DcMotor> driveMotors;

    public final IMU imu;
    public OpenCvWebcam camera1;
    public WebcamName webcamName;

    public SparkFunOTOS opticalSensor;
    HardwareMap hwmap;


    public static enum MOTORS {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
    }

    public RobotClass(HardwareMap hwmap) {
        this.hwmap = hwmap;

        imu = hwmap.get(IMU.class, "imu");
        driveMotors = new HashMap<>();
        driveMotors.put(MOTORS.FRONT_LEFT, hwmap.get(DcMotor.class, "frontLeft"));
        driveMotors.put(MOTORS.FRONT_RIGHT, hwmap.get(DcMotor.class, "frontRight"));
        driveMotors.put(MOTORS.BACK_LEFT, hwmap.get(DcMotor.class, "backLeft"));
        driveMotors.put(MOTORS.BACK_RIGHT, hwmap.get(DcMotor.class, "backRight"));

        drivetrain = new MecanumDrive(driveMotors, this);

        opticalSensor = hwmap.get(SparkFunOTOS.class, "opticalSensor");
        webcamName = hwmap.get(WebcamName.class, "Webcam 1");
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double getHeading() {
        Orientation Theta = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        return Theta.thirdAngle;
    }

    public double getRotationRate() {
        AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return velocity.zRotationRate;
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void setDirection() {
        driveMotors.get(MOTORS.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotors.get(MOTORS.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotors.get(MOTORS.FRONT_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotors.get(MOTORS.BACK_RIGHT).setDirection(DcMotorSimple.Direction.FORWARD);
    }



    public void stopAndReset() {
        driveMotors.get(MOTORS.FRONT_LEFT).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors.get(MOTORS.FRONT_LEFT).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //drivetrain.driveMotors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //drivetrain.driveMotors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //drivetrain.driveMotors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
