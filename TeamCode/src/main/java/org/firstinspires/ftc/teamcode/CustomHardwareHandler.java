package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.teamcode.movement.imu.SimpsonIntegrator;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.structures.PIDFController;
//import org.firstinspires.ftc.teamcode.structures.SlidePosition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CustomHardwareHandler {

    // Define your motors here
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor linearSlideRight;
    private DcMotor linearSlideLeft;
    private DcMotor actuatorLeft;
    private DcMotor actuatorRight;

    // Define your servo variables
    private Servo railLaunch;
    private Servo outputDoor;
    private Servo conveyorBelt;
    private Servo intake;

    private double axial;

    private double yaw;

    private double lateral;

    private double backLeftPower;

    private double backRightPower;

    private double frontLeftPower;

    private double frontRightPower;

    // Constructor
    public CustomHardwareHandler(HardwareMap hardwareMap) {
        // Initialize your motors by retrieving them from the hardware map
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        linearSlideRight = hardwareMap.dcMotor.get("linearSlideRight");
        linearSlideLeft = hardwareMap.dcMotor.get("linearSlideLeft");
        actuatorLeft = hardwareMap.dcMotor.get("actuatorLeft");
        actuatorRight = hardwareMap.dcMotor.get("actuatorRight");

        railLaunch = hardwareMap.servo.get("railLaunch");
        outputDoor = hardwareMap.servo.get("outputDoor");
        conveyorBelt = hardwareMap.servo.get("conveyorBelt");
        intake = hardwareMap.servo.get("intake");

        // Set the direction for the "left" motors to reverse
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        actuatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor run modes to RUN_WITHOUT_ENCODER
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        actuatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        actuatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior to BRAKE for front and rear motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set zero power behavior, max power, etc. as needed
        // ...
    }

    // Other methods for controlling the motors, setting power, etc.
    // ...
}
