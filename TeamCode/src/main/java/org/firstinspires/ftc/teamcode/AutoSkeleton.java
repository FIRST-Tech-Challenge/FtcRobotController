package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Skelly Don't Run")
public class AutoSkeleton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable

    private final int MAX_TARGET_LIFT = 2825;        // Constaints on extending mechanisms
    private final int MAX_EXTENSION_LENGTH = 500;    // ^

    private int TargetLift = 700;                    // Target variables for extending mechanisms
    private int ExtensionTarget = 0;

    private double LiftPower = .9;
    private double ExtensionPower = .75;
    private boolean IntakeClawClosed = false;         // claw holder variable
    private boolean OuttakeClawClosed = false;        // ^

    private ElapsedTime Transfer_Time = new ElapsedTime();
    private ElapsedTime ClawTime = new ElapsedTime();
    private ElapsedTime ClawDelay = new ElapsedTime();

    public double LeftServo;                   // Intake variables
    public double RightServo;
    private double V4Bpos = 1;
    private double Flex = 0;
    private double Yaw = 0;

        //*********************************** MOTORS ************************************************
        DcMotor FrontRight = hardwareMap.dcMotor.get("Front Right");   // Chub Port 0 // Gamepad 1
        DcMotor BackRight = hardwareMap.dcMotor.get("Back Right");     // Chub Port 1 // Left Stick For Moving
        DcMotor FrontLeft = hardwareMap.dcMotor.get("Front Left");     // Chub Port 2 // Right Stick For Turning
        DcMotor BackLeft = hardwareMap.dcMotor.get("Back Left");       // Chub Port 3


        DcMotor IntakeRight = hardwareMap.dcMotor.get("Intake Right"); // Ehub Port 0 // X Button To Position Automatically? // Joystick Up And Down?
        DcMotor IntakeLeft = hardwareMap.dcMotor.get("Intake Left");   // Ehub Port 1 // ----------------------------------
        DcMotor RightLift = hardwareMap.dcMotor.get("Right Lift");     // Ehub Port 2 // Triangle Button To Delivery Position
        DcMotor LeftLift = hardwareMap.dcMotor.get("Left Lift");       // Ehub Port 3 // ------------------------------------

        //*********************************** SERVOS ************************************************
        Servo IntakeClaw = hardwareMap.servo.get("Intake Claw");              // Chub Port 0 // O Button
        Servo RightIntakeWrist = hardwareMap.servo.get("Right Intake Wrist"); // Chub Port 1 // Increments Using Dpad Side Buttons?
        Servo LeftIntakeWrist = hardwareMap.servo.get("Left Intake Wrist");   // Chub Port 2 // Ideally Stick Controlled
        Servo IntakeV4B = hardwareMap.servo.get("Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
       // Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
      //  Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

        Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
        Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
        Servo OuttakeV4B = hardwareMap.servo.get("Outtake V4B");   // Ehub Port 2 // Preset With Triangle
       // Servo LeftHook = hardwareMap.servo.get("Left Hook");                  // Ehub Port 4 // Both Players Press A Button TBD Which

        LeftServo = Flex - (.5 * Yaw);
        RightServo = Flex + (.5 * Yaw);

        //****************************** REVERSE MOTORS *****************************************************

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns

        LeftLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

        //****************************** RESET ENCODERS *******************************************************
        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // Resets the position so it sets it's current position to 0
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Resets the position so it sets it's current position to 0

        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Resets the position so it sets it's current position to 0
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Resets the position so it sets it's current position to 0

        //****************************** SET DEFAULT TARGET POSITION ***********************************************
        LeftLift.setTargetPosition(0);     // Makes sure it starts at the set 0
        RightLift.setTargetPosition(0);    // Makes sure it starts at the set 0

        IntakeRight.setTargetPosition(0);  // Makes sure it starts at the set 0
        IntakeLeft.setTargetPosition(0);   // Makes sure it starts at the set 0

        //****************************** SET MODE TO RUN_TO_POSITION ***************************************************
        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Sets the mode so we can say to move the motor a certain amount of ticks
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Sets the mode so we can say to move the motor a certain amount of ticks

        IntakeRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Sets the mode so we can say to move the motor a certain amount of ticks
        IntakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);    // Sets the mode so we can say to move the motor a certain amount of ticks

        //****************************** SET MOTORS TO BRAKE MODE *****************************************************
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // Sets the motor to be locked when stopped
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // Sets the motor to be locked when stopped
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // Sets the motor to be locked when stopped
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Sets the motor to be locked when stopped

        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // Sets the motor to be locked when stopped
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Sets the motor to be locked when stopped

        //***************************** RESET SERVOS ***********************************************************
        IntakeClaw.setPosition(0);    // Closes Intake Claw
        OuttakeClaw.setPosition(0);   // Closes Outtake Claw

        LeftServo = Math.max(0, Math.min(1, Flex - (.5 * Yaw)));
        RightServo = Math.max(0, Math.min(1, Flex + (.5 * Yaw)));

        LeftIntakeWrist.setPosition(LeftServo);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(RightServo);   // Sets the intake wrist to the starting position // Left is 0 Right is 1
        IntakeV4B.setPosition(.78);   // Sets the intake virtual four bar to the starting position
        //RightIntakeV4B.setPosition(1);  // Sets the intake virtual four bar to the starting position

        OuttakeWrist.setPosition(0);    // Sets the outtake wrist to the starting position

        OuttakeV4B.setPosition(1);  // Sets the outtake virtual four bar to the starting position
        //RightOuttakeV4B.setPosition(0); // Sets the outtake virtual four bar to the starting position

     //   LeftHook.setPosition(0);    // Sets the left hook to the starting position
     //   RightHook.setPosition(0);   // Sets the right hook to the starting position


        waitForStart();
        while(opModeIsActive()){
            //Drive To Submersible

            //Deliver Specimen

            //Go To The Right And Push The Sample Back To Observation Zone

            //Pick Up Specimen Off Wall And Repeat
        }
    }
}
