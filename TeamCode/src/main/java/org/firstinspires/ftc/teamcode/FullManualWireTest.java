package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FullManualWireTest extends LinearOpMode {

    private double LiftPower = 0;
    private double ExtensionPower = 0;


    private ElapsedTime IntakeClawTime = new ElapsedTime();  // Sets up a timer to keep track since last time the intake claw was used
    private ElapsedTime OuttakeClawTime = new ElapsedTime(); // Sets up a timer to keep track since last time the outtake claw was used


    //****************************** BOOLEANS ********************************************************************************
    private boolean IntakeClawClosed = false;      // Intake claw state variable
    private boolean OuttakeClawClosed = false;     // Outtake claw state variable
    private boolean OuttakeActive = false;         // Keeps track of the lift to see if it is extended or not


    @Override
    public void runOpMode() {
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
        //Servo LeftIntakeV4B = hardwareMap.servo.get("Left Intake V4B");       // Chub Port 4 // --------------------------
        //Servo RightHook = hardwareMap.servo.get("Right Hook");                // Chub Port 5 // Linked To LeftHook Activated At The Same Time

        Servo OuttakeClaw = hardwareMap.servo.get("Outtake Claw");            // Ehub Port 0 // If Slides Up O Activates This Claw
        Servo OuttakeWrist = hardwareMap.servo.get("Outtake Wrist");          // Ehub Port 1 // Preset To Go To Delivery Position With Triangle
        Servo OuttakeV4B = hardwareMap.servo.get("Outtake V4B");   // Ehub Port 2 // Preset With Triangle
        // Servo LeftOuttakeV4B = hardwareMap.servo.get("Left Outtake V4B");     // Ehub Port 3 // --------------------
        //Servo LeftHook = hardwareMap.servo.get("Left Hook");                  // Ehub Port 4 // Both Players Press A Button TBD Which

        //****************************** REVERSE MOTORS *****************************************************

        IntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);   // Reverses the direction the motor turns

        RightLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

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

        IntakeV4B.setPosition(1);   // Sets the intake virtual four bar to the starting position
        //RightIntakeV4B.setPosition(1);  // Sets the intake virtual four bar to the starting position

        OuttakeWrist.setPosition(0);    // Sets the outtake wrist to the starting position

        OuttakeV4B.setPosition(.5);  // Sets the outtake virtual four bar to the starting position
        //RightOuttakeV4B.setPosition(0); // Sets the outtake virtual four bar to the starting position

        //LeftHook.setPosition(0);    // Sets the left hook to the starting position
        //RightHook.setPosition(0);   // Sets the right hook to the starting position

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_up) { //Backwards
                FrontLeft.setPower(1);
            } else {
                FrontLeft.setPower(0);
            }
            if (gamepad2.dpad_right) { // Forwards
                FrontRight.setPower(1);
            } else {
                FrontRight.setPower(0);
            }
            if (gamepad2.dpad_down) {   //Forwards
                BackRight.setPower(1);
            } else {
                BackRight.setPower(0);
            }
            if (gamepad2.dpad_left) { // Backwards
                BackLeft.setPower(1);
            } else {
                BackLeft.setPower(0);
            }

            double Ly = gamepad2.left_stick_y;         // Remember, this is reversed!
            double Ry = gamepad2.right_stick_y;         // Remember, this is reversed!

            LiftPower = Ly;
            ExtensionPower = Ry;

            //RightLift.setPower(LiftPower); Un-comment this if values are ok from telemetry
            //LeftLift.setPower(LiftPower);  Un-comment this if values are ok from telemetry

            //IntakeLeft.setPower(ExtensionPower); Un-comment this if values are ok from telemetry
            //IntakeRight.setPower(ExtensionPower); Un-comment this if values are ok from telemetry

            telemetry.addData("Lift Power", LiftPower)
                    .addData("Extension Power", ExtensionPower);

            //**************************** CLAW CONTROLS *********************************************************8
            // Opens the outtake claw
            if (gamepad2.x && OuttakeClawTime.seconds() >= .3 && OuttakeClawClosed) { // If the B button was pressed, Outtake is extended and it has been more than .3 seconds since the outtake claw has been used
                OuttakeClawTime.reset();    // Reset the timer since the outtake claw was just used
                OuttakeClaw.setPosition(0); // Opens the outtake claw
                OuttakeClawClosed = false;  // Since the outtake claw was opened we change this to stay accurate
            } else if (gamepad2.x && OuttakeClawTime.seconds() >= .3 && !OuttakeClawClosed) {
                OuttakeClawTime.reset();
                OuttakeClaw.setPosition(1);
                OuttakeClawClosed = true;
            }
            // Closes the intake claw
            if (gamepad2.b && IntakeClawTime.seconds() >= .3 && !IntakeClawClosed) { // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is open
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(1);  // Closes the intake claw
                IntakeClawClosed = true;    // Since the intake claw was closed we change this to stay accurate
            }
            // Opens the intake claw
            else if (gamepad2.b && IntakeClawTime.seconds() >= .3 && IntakeClawClosed) {  // If the B button was pressed, Outtake is retracted, It has been more than .3 seconds since the intake claw has been used and the intake claw is closed
                IntakeClawTime.reset();     // Reset the timer since the intake claw was just used
                IntakeClaw.setPosition(0);  // Opens the intake claw
                IntakeClawClosed = false;   // Since the intake claw was opened we change this to stay accurate
            }

          /*  if (gamepad2.left_trigger > 0) {
                LeftIntakeV4B.setPosition(0);
                RightIntakeV4B.setPosition(0);
            } else {
                LeftIntakeV4B.setPosition(1);
                RightIntakeV4B.setPosition(1);
            }

            if (gamepad2.right_bumper) { OuttakeWrist.setPosition(1); } else{ OuttakeWrist.setPosition(0);}

            if (gamepad2.right_trigger > 0){
                RightOuttakeV4B.setPosition(1);
                LeftOuttakeV4B.setPosition(1);
            } else{
                RightOuttakeV4B.setPosition(0);
                LeftOuttakeV4B.setPosition(0);
            }

            if (gamepad2.y){ RightHook.setPosition(1); LeftHook.setPosition(1); }
            else{ RightHook.setPosition(0); LeftHook.setPosition(0); }
        }
    }*/
        }
    }
}