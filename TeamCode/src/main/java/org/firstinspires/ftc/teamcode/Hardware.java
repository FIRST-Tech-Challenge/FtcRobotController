package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware
{
    //Drive Motors Declaration
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    //Transfer Motors and Servos Declaration
    public DcMotorEx TransferM1;
    public DcMotorEx TransferM2;
    public CRServo TransferCR1;
    public CRServo TransferCR2;

    //Intake Servos Declaration
    public CRServo counteroller;

    public DcMotor intakeMotor;

    //Deposit Servos Declaration
    public Servo depositServoOne;
    public Servo depositServoTwo;

    //Odometry Helper Class Variables
    public double x = 0, y = 0, theta = 0;
    public static LinearOpMode currentOpMode;

    public DcMotorEx leftOdom, rightOdom, centerOdom;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    //used in our odo but not RoadRunner classes
    public static final double ODOM_TICKS_PER_IN = 1831.471685;
    public static double trackwidth = 10.976;

    public Hardware(HardwareMap hardwareMap)
    {
        //Drive Motor Initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        //Odom
        leftOdom = hardwareMap.get(DcMotorEx.class, "Front Left");
        rightOdom = hardwareMap.get(DcMotorEx.class, "Front Right");
        centerOdom = hardwareMap.get(DcMotorEx.class, "Back Right");

        //Transfer Motor Config -- Raise motor
        TransferM1 = hardwareMap.get(DcMotorEx.class, "Transfer Motor 1");
        TransferM1 = hardwareMap.get(DcMotorEx.class, "Transfer Motor 2");

        //Intake Config - Transfer servos spin with intake servos/motors
        counteroller = hardwareMap.get(CRServo.class, "Intake Servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        TransferCR1 = hardwareMap.get(CRServo.class, "Transfer Servo 1");
        TransferCR2 = hardwareMap.get(CRServo.class, "Transfer Servo 2");

        //Deposit Servo Config
        depositServoOne = hardwareMap.get(Servo.class, "Right Deposit");
        depositServoTwo = hardwareMap.get(Servo.class, "Left Deposit");
    }

    //robot-oriented drive method
    public void robotODrive(double forward, double sideways, double rotation)
    {
        //adds all the inputs together to get the number to scale it by
        double scale = Math.abs(rotation) + Math.abs(forward) + Math.abs(sideways);

        //Scales the inputs between 0-1 for the setPower() method
        if (scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }

        //Zeroes out and opposing or angular force from the Mechanum wheels
        frontLeft.setPower(forward - rotation - sideways);
        backLeft.setPower(forward - rotation + sideways);
        frontRight.setPower(forward + rotation + sideways);
        backRight.setPower(forward + rotation - sideways);
    }}