package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO - adjust static variables for new mecanum drive hardware
// TODO - We have the values from the touchpad, still need to implement it into the wrist

@TeleOp(name = "TeleOp Skeleton")

public class TeleOpTesting extends LinearOpMode {
    //*******************************OUTDATED*****************************************************************************
    //static final double COUNTS_PER_MOTOR_REV = 288;    // eg: TETRIX Motor Encoder
    //static final double DRIVE_GEAR_REDUCTION = 1;      // This is < 1.0 if geared UP
    //static final double WHEEL_DIAMETER_INCHES = .5;    // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    //*********************************************************************************************************************

    //****************************** FINAL ********************************************************************************
    // Final variables (Meaning they don't change)
    static final int MAX_LIFT_HEIGHT = 1000; // Max outtake lift height // Needs Tweaked
    static final int MAX_EXTENSION_LIMIT = 1000; // Max intake extension length // Needs Tweaked

    //****************************** TIMERS ********************************************************************************
    // Sets up timers to let the robot be able to know how much time has passed
    private ElapsedTime IntakeClawTime = new ElapsedTime();  // Sets up a timer to keep track since last time the intake claw was used
    private ElapsedTime OuttakeClawTime = new ElapsedTime(); // Sets up a timer to keep track since last time the outtake claw was used
    private ElapsedTime xButtonTime = new ElapsedTime(); // Sets up a timer to keep track since the last time the X button was pressed

    //Variables (meaning they can change)
    //****************************** DOUBLES ********************************************************************************
    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable
    private double liftPower = 1;          // declare lift motor power variable
    private double ExtensionPower = 1;     // declare extension power variable
    private double TouchPadInput = .5;     // Keeps track of finger position on the touchpad

    //****************************** INTEGERS ********************************************************************************
    private int precision = 2;                                  // chassis motor power reduction factor 1=full 2=1/2 power 4=1/4 power
    private int LiftTarget = 0;                                 // Lift target position variable
    private int ExtensionTarget = 0;                            // Extension target position variable
    private int HalfIntakeExtension = MAX_EXTENSION_LIMIT / 2;  // Sets the default for the intake extension preset to half of the full extension

    //****************************** BOOLEANS ********************************************************************************
    private boolean IntakeClawClosed = false;      // Intake claw state variable
    private boolean OuttakeClawClosed = false;     // Outtake claw state variable
    private boolean OuttakeActive = false;         // Keeps track of the lift to see if it is extended or not

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

        //****************************** REVERSE MOTORS *****************************************************
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns

        IntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns

        RightLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

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

        LeftIntakeWrist.setPosition(.5);    // Sets the intake wrist to the starting position // Left is 0 Right is 1
        RightIntakeWrist.setPosition(.5);   // Sets the intake wrist to the starting position // Left is 0 Right is 1

        LeftIntakeV4B.setPosition(0);   // Sets the intake virtual four bar to the starting position
        RightIntakeV4B.setPosition(0);  // Sets the intake virtual four bar to the starting position

        OuttakeWrist.setPosition(0);    // Sets the outtake wrist to the starting position

        LeftOuttakeV4B.setPosition(0);  // Sets the outtake virtual four bar to the starting position
        RightOuttakeV4B.setPosition(0); // Sets the outtake virtual four bar to the starting position

        LeftHook.setPosition(0);    // Sets the left hook to the starting position
        RightHook.setPosition(0);   // Sets the right hook to the starting position

        waitForStart();                               // Waits for us to hit play before continuing

        while (opModeIsActive()) {                    // Loops through everything until we hit stop

            //*************************** DRIVE CONTROLS **************************************************
            // check for driving input
            double y = gamepad1.left_stick_y;         // Remember, this is reversed!
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.left_stick_x;       // Measures turning
            if(gamepad1.right_trigger >= 0.75) {      // Checks if the Right Trigger was pressed and if so it continues the stuff in the brackets
                y = -gamepad1.left_stick_y;           // Remember, this is reversed!
                x = gamepad1.right_stick_x * 1.1;    // Counteract imperfect strafing
                rx = gamepad1.left_stick_x;          // Measures turning
            }
            
            //**************************** INTAKE SLIDES ******************************************************
            //Extends intake
            if (gamepad2.a && !OuttakeActive){  // If the A button is pressed and the outtake is retracted
                ExtensionTarget = HalfIntakeExtension;    // Sets the ExtensionTarget variable to the default extension length
                RightIntakeV4B.setPosition(1);  // Sets the intake virtual four bar to pickup position
                LeftIntakeV4B.setPosition(1);   // Sets the intake virtual four bar to pickup position
            }
            // Retracts intake
            else if (gamepad2.x && !OuttakeActive && xButtonTime.seconds() >= .3){  // If the X button is pressed, outtake is retracted and it has been more than .3 seconds sine the X button was pressed last
                xButtonTime.reset();    // Restarts the timer since X was just pressed
                RightIntakeV4B.setPosition(0);  // Sets the intake virtual four bar to transfer position
                LeftIntakeV4B.setPosition(0);   // Sets the intake virtual four bar to transfer position
                ExtensionTarget = 1;    // Return Intake To Start Position
            }

            //**************************** OUTTAKE SLIDES **********************************************************
            //Extends Outtake
            if (gamepad2.y){                    // If the Y button is pressed
                OuttakeActive = true;           // Since the outtake is moving we set it to true to keep it accurate
                OuttakeClaw.setPosition(1);     // Closes the outtake claw
                OuttakeClawClosed = true;       // Sets the variable that keeps track of the claw to be accurate
                LiftTarget = 800;               // Sets the Lift target to 800 ticks
                ExtensionTarget = 1;            // Sets the extension target to 1 tick
                LeftOuttakeV4B.setPosition(1);  // Sets the outtake virtual four bar to delivery position
                RightOuttakeV4B.setPosition(1); // Sets the outtake virtual four bar to delivery position
                OuttakeWrist.setPosition(1);    // Sets the outtake wrist to delivery position
            }
            //Retracts Outtake
            if (gamepad2.x && OuttakeActive && xButtonTime.seconds() >= .3){  // If the X button was pressed, Outtake is extended and it has been more than .3 seconds since the X button was last pressed
                xButtonTime.reset();            // Resets the timer since X was just pressed
                OuttakeActive = false;          // Since the outtake is retracted we set it to false to keep it accurate
                LiftTarget = 1;                 // Sets the lift target to 1 tick
                LeftOuttakeV4B.setPosition(0);  // Sets the outtake virtual four bar to transfer position
                RightOuttakeV4B.setPosition(0); // Sets the outtake virtual four bar to transfer position
                OuttakeWrist.setPosition(0);    // Sets the outtake wrist to delivery position
            }

            //**************************** CLAW CONTROLS *********************************************************8
            // Opens the outtake claw
            if (gamepad2.b && OuttakeActive && OuttakeClawTime.seconds() >= .3 && OuttakeClawClosed){ // If the B button was pressed, Outtake is extended and it has been more than .3 seconds since the outtake claw has been used
                OuttakeClawTime.reset();    // Reset the timer since the outtake claw was just used
                OuttakeClaw.setPosition(0); // Opens the outtake claw
                OuttakeClawClosed = false;  // Since the outtake claw was opened we change this to stay accurate
            }
            else if (gamepad2.b && OuttakeActive && OuttakeClawTime.seconds() >= .3 && !OuttakeClawClosed){
                OuttakeClawTime.reset();
                OuttakeClaw.setPosition(1);
                OuttakeClawClosed = true;
            }
            // Closes the intake claw
            else if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && !IntakeClawClosed){ // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is open
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(1);  // Closes the intake claw
                IntakeClawClosed = true;    // Since the intake claw was closed we change this to stay accurate
            }
            // Opens the intake claw
            else if (gamepad2.b && !OuttakeActive && IntakeClawTime.seconds() >= .3 && IntakeClawClosed){  // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is closed
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(0);  // Opens the intake claw
                IntakeClawClosed = false;   // Since the intake claw was opened we change this to stay accurate
            }

            //**************************** INTAKE WRIST ************************************************************
            // Intake Wrist
            if (IntakeLeft.getCurrentPosition() > 200 && gamepad1.touchpad_finger_1){   // The intake is extended past 200 ticks and a finger is on the touchpad
                    TouchPadInput = (gamepad1.touchpad_finger_1_x + 1) / 2;     // This is taking a range from -1 - 1 and converting it to a range of 0 - 1 and saving it to a variable
            }
            else if (IntakeLeft.getCurrentPosition() < 200) {   // The intake is extended below 200 ticks
                TouchPadInput = .5;     // If the above statement is false then it defaults the value to .5
            }


            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);    // calculate motor movement math and adjust according to lift height or manual precision mode selection

            // check for Turbo or Precision Mode
            if (gamepad1.left_bumper) { // Left bumper is being pressed
                precision = 1;              // set speed to full power - TURBO MODE
            } else if (gamepad1.right_bumper) { // Right bumper is being pressed
                precision = 4;              // set speed to 1/4 power - PRECISION MODE
            } else {
                precision = 2;              // reset default speed to half power
            }

            // calculate motor power
            denominator = denominator * precision;          // this adjusts motor speed to the desired precision level
            frontLeftPower = (y + x + rx) / denominator;    // Does math to convert stick movement to motor powers
            backLeftPower = (y - x + rx) / denominator;     // Does math to convert stick movement to motor powers
            frontRightPower = (y - x - rx) / denominator;   // Does math to convert stick movement to motor powers
            backRightPower = (y + x - rx) / denominator;    // Does math to convert stick movement to motor powers

            //************************* MANUAL SLIDE CONTROLS *******************************************
            // Manual Intake
            if ( gamepad2.dpad_up && ExtensionTarget < MAX_EXTENSION_LIMIT - 20){   // Up on the dpad is pressed and the Extension target is less than the max extension limit - 20
                ExtensionTarget = ExtensionTarget + 10; // Takes the extension target and adds 10
            }
            else if ( gamepad2.dpad_down && ExtensionTarget > 20){  // Down on the dpad is pressed and the Extension target is greater than 20
                ExtensionTarget = ExtensionTarget - 10; // Takes the extension target and subtracts 10
            }
            // Manual lift up and down
            if (gamepad2.dpad_up && LiftTarget < MAX_LIFT_HEIGHT - 10) {// If the dpad up is pressed and the lift target is not above the max height
                LiftTarget = LiftTarget + 10;                           // This adds lift target +10 to the current lift target having the lift move up slowly
            }
            else if (gamepad2.dpad_down && LiftTarget >= 10) {          // If the dpad down is pressed and the lift target is higher than 10
                LiftTarget = LiftTarget - 10;                           // This subtracts lift target -10 from the current lift target having the lift move up slowly
            }

            //************************* MOTOR POWER ASSIGNMENTS ****************************************
            // Intake power and movement
            if (!(ExtensionTarget > MAX_LIFT_HEIGHT)){          // Extension target < the max height
                IntakeLeft.setTargetPosition(ExtensionTarget);  // Sets the Intake Motors to a synced position
                IntakeRight.setTargetPosition(ExtensionTarget); // Sets the Intake Motors to a synced position
                IntakeLeft.setPower(ExtensionPower);            // Sets the Motor Power to ExtensionPower Declared Above
                IntakeRight.setPower(ExtensionPower);           // Sets the Motor Power to ExtensionPower Declared Above
            }
            // Lift Power and movement
            if (!(LiftTarget > MAX_LIFT_HEIGHT)) {          // If the lift target height is < the max height
                RightLift.setTargetPosition(LiftTarget);    // Sets the right lift motor to turn until it's ticks = lift target
                LeftLift.setTargetPosition(LiftTarget);     // Sets the left lift motor to turn until it's ticks = lift target
                RightLift.setPower(liftPower);              // Sets the power to the Right lift motor
                LeftLift.setPower(liftPower);               // Sets the power to the left lift motor
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
