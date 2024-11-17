package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pending - change variable names to 2025 robot
// TODO - adjust static variables for new mecanum drive hardware
// TODO - fix Mecanum motor assignments and code for normal operations

@TeleOp(name = "TeleOp Skeleton")


// This opmode has framework for all systems included

public class TeleOpTesting extends LinearOpMode {
    // Final variables (Meaning they don't change)
    //*******************************OUTDATED*****************************************************************************
    //static final double COUNTS_PER_MOTOR_REV = 288;    // eg: TETRIX Motor Encoder
    //static final double DRIVE_GEAR_REDUCTION = 1;      // This is < 1.0 if geared UP
    //static final double WHEEL_DIAMETER_INCHES = .5;    // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    //*********************************************************************************************************************

    static final int MAX_LIFT_HEIGHT = 1000; //Start Low and test and adjust from there

    static final int MAX_EXTENSION_LIMIT = 1000;

    //Creates Timer cvariables to keep track of time passed
    private ElapsedTime IntakeClawTime = new ElapsedTime();        // Sets up timer functions
    private ElapsedTime OuttakeClawTime = new ElapsedTime();
    private ElapsedTime LiftTime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();          // Sets up a timer function
    private ElapsedTime runtime2 = new ElapsedTime();         // Sets up a timer function
    private ElapsedTime xButtonTime = new ElapsedTime();

    //Variables (meaning they can change)
    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable
    private int precision = 2;             // chassis motor power reduction factor 1=full 2=1/2 power 4=1/4 power
    private double liftPower = 1;          // declare lift motor power variable *******
    private double ExtensionPower = 1;
    private boolean IntakeClawClosed = false;     // Claw state variable
    private boolean OuttakeClawClosed = false;     // Claw state variable
    private boolean OuttakeActive = false;
    private int LiftTarget = 0;            // Lift target position variable

    private int ExtensionTarget = 0;
    private int HalfIntakeExtension = MAX_EXTENSION_LIMIT / 2;
    private double TouchPadInput = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

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
        Servo RightIntakeV4B = hardwareMap.servo.get("Right Intake V4B");     // Chub Port 3 // Preset To Swing Out With X
        Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
        Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

        Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
        Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
        Servo RightOuttakeV4B = hardwareMap.servo.get("Right Outtake V4B");   // Ehub Port 2 // Preset With Triangle
        Servo LeftOuttakeV4B = hardwareMap.servo.get("Left Outtake V4B");     // Ehub Port 3 // --------------------
        Servo LeftHook = hardwareMap.servo.get("Left Hook");                  // Ehub Port 4 // Both Players Press A Button TBD Which

        // Reverse the right side motors

// TODO - this was a bandaid fix.........

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);            // Reverses the direction the motor turns
        // FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);            // Reverses the direction the motor turns
        //BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//          LeftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // Resets the position so it sets it's current position to 0
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Resets the position so it sets it's current position to 0

        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift.setTargetPosition(0);     // Makes sure it starts at the set 0
        RightLift.setTargetPosition(0);    // Makes sure it starts at the set 0

        IntakeRight.setTargetPosition(0);
        IntakeLeft.setTargetPosition(0);

        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Sets the mode so we can say to move the motor a certain amount of ticks
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Sets the mode so we can say to move the motor a certain amount of ticks

        IntakeRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor zero power behavior

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locked when stopped
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locked when stopped
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locks when stopped
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locked when stopped

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locked when stopped
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locked when stopped

        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeClaw.setPosition(0);                         // Closes Intake Claw
        OuttakeClaw.setPosition(0);                        // Closes Outtake Claw

        LeftIntakeWrist.setPosition(.5); // Left is 0 Right is 1
        RightIntakeWrist.setPosition(.5);

        LeftIntakeV4B.setPosition(0);
        RightIntakeV4B.setPosition(0);

        OuttakeWrist.setPosition(0);

        RightOuttakeV4B.setPosition(0);
        LeftOuttakeV4B.setPosition(0);

        LeftHook.setPosition(0);
        RightHook.setPosition(0);

        waitForStart();                               // Waits for us to hit play before continuing

        while (opModeIsActive()) {                    // Loops through everything until we hit stop

            // check for driving input
            double y = gamepad1.left_stick_y;         // Remember, this is reversed!
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.left_stick_x;       // Measures turning
            if(gamepad1.right_trigger >= 0.75) {      // Checks if the Right Trigger was pressed and if so it continues the stuff in the brackets
                y = -gamepad1.left_stick_y;           // Remember, this is reversed!
                x = gamepad1.right_stick_x * 1.1;    // Counteract imperfect strafing
                rx = gamepad1.left_stick_x;          // Measures turning
            }

            if (gamepad2.a && !OuttakeActive){
                ExtensionTarget = HalfIntakeExtension;    // If button A was pushed extend intake claw
            }
            else if (gamepad2.x && !OuttakeActive && xButtonTime.seconds() >= .3){
                xButtonTime.reset();
                ExtensionTarget = 1;                // Return Intake To Start Position

            }

            if (gamepad2.y){
                OuttakeActive = true;           // Outtake Claw Closed Outtake Delivery Preset
                OuttakeClaw.setPosition(1);
                OuttakeClawClosed = true;
                LiftTarget = 800;
                RightIntakeV4B.setPosition(1);
                LeftIntakeV4B.setPosition(1);
                OuttakeWrist.setPosition(1);
            }
            if (gamepad2.x && OuttakeActive && xButtonTime.seconds() >= .3){        // Transfer Preset
                xButtonTime.reset();
                OuttakeActive = false;
                LiftTarget = 1;
                RightIntakeV4B.setPosition(0);
                LeftIntakeV4B.setPosition(0);
                OuttakeWrist.setPosition(0);
            }
            if (gamepad2.b && OuttakeActive && OuttakeClawTime.seconds() >= .3){ // Opens outtake claw
                OuttakeClawTime.reset();
                OuttakeClaw.setPosition(0);
                OuttakeClawClosed = false;
            }
            else if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && !IntakeClawClosed){ // closes intake claw
                IntakeClawTime.reset();
                IntakeClaw.setPosition(1);
                IntakeClawClosed = true;
            }
            else if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && IntakeClawClosed){ // opens intake claw
                IntakeClawTime.reset();
                IntakeClaw.setPosition(0);
                IntakeClawClosed = false;
            }


            // Intake Wrist
            if (IntakeLeft.getCurrentPosition() > 200 && gamepad1.touchpad_finger_1){
                    TouchPadInput = (gamepad1.touchpad_finger_1_x + 1) / 2;
            }
            else {
                TouchPadInput = .5;
            }

            // Manual Intake
            if ( gamepad2.dpad_up && ExtensionTarget < MAX_EXTENSION_LIMIT - 20){
                ExtensionTarget = ExtensionTarget + 10;
            }
            else if ( gamepad2.dpad_down && ExtensionTarget > 20){
                ExtensionTarget = ExtensionTarget - 10;
            }


            telemetry.update();//Updates the telemetry on the driver hub

            // calculate motor movement math and adjust according to lift height or manual precision mode selection

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // check for Turbo or Precision Mode

            if (gamepad1.left_bumper) {
                precision = 1;              // set speed to full power - TURBO MODE
            } else if (gamepad1.right_bumper) {
                precision = 4;              // set speed to 1/4 power - PRECISION MODE
            } else {
                precision = 2;              // reset default speed to half power
            }

            // calculate motor power

            denominator = denominator * precision;          // this adjusts motor speed to the desired precision level
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            // check for lift movement input




            if (gamepad2.dpad_up && LiftTarget < MAX_LIFT_HEIGHT - 10) {// If the dpad up is pressed and the lift target is not above the max height
                LiftTarget = LiftTarget + 10;                           // This adds lift target +10 to the current lift target having the lift move up slowly
            }
            if (gamepad2.dpad_down && LiftTarget >= 10) {               // If the dpad down is pressed and the lift target is higher than 10
                LiftTarget = LiftTarget - 10;                           // This subtracts lift target -10 from the current lift target having the lift move up slowly
            }


            if (!(ExtensionTarget > MAX_LIFT_HEIGHT)){ // Extension target < the max height
                IntakeLeft.setTargetPosition(ExtensionTarget); // Sets the Intake Motors to a synced position
                IntakeRight.setTargetPosition(ExtensionTarget); // Sets the Intake Motors to a synced position
                IntakeLeft.setPower(ExtensionPower); // Sets the Motor Power to ExtensionPower Declared Above
                IntakeRight.setPower(ExtensionPower); // Sets the Motor Power to ExtensionPower Declared Above
            }


            //Lift Power and movement
            if (!(LiftTarget > MAX_LIFT_HEIGHT)) {           // If the lift target height is < the max height
                RightLift.setTargetPosition(LiftTarget);// Sets the right lift motor to turn until it's ticks = lift target
                LeftLift.setTargetPosition(LiftTarget); // Sets the left lift motor to turn until it's ticks = lift target
                RightLift.setPower(liftPower);          // Sets the power to the Right lift motor
                LeftLift.setPower(liftPower);           // Sets the power to the left lift motor
            }


            // issue Drive Wheels motor power

            FrontLeft.setPower(frontLeftPower);    // Sets the front left wheel's power
            BackLeft.setPower(backLeftPower);     // Sets the back left wheel's power
            FrontRight.setPower(frontRightPower);  // Sets the front right wheel's power
            BackRight.setPower(backRightPower);   // Sets the back right wheel's power


            telemetry.addData("Right Lift Height", RightLift.getCurrentPosition());    // Displays the ticks of the right lift motor on the driver hub
            telemetry.update();                      // Updates the telemetry so we have live readings of the lift height
        }

    }
}
