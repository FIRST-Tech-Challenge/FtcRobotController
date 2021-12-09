package org.firstinspires.ftc.teamcode;


import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: First Program", group="Knightrix")
public class drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor spinnerIntake = null;
    private DcMotor spinnerArm = null;
    TouchSensor BoxTouch;
//    Servo servo;
    Servo servoFlip;
    CRServo contServo;

    // Variables
    int Drop_Rotation = 160;
    int Drop_Range = 30;
    int Pickup_Hover = 10;
    boolean dropping = false;
    double armTargetPos = 0;
    double armPos = 0;
    static double Right_TrigPosition;
    static double spinPower = 0;

    //right trigger move one motor more depending on 1 or -1 values (Range.clip())
    //encoder used to move intake
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone)
        // Adding touch sensor as hardware program.
        BoxTouch = hardwareMap.get(TouchSensor.class, "Box_Touch");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        spinnerIntake = hardwareMap.get(DcMotor.class, "spinner_intake");
        spinnerArm = hardwareMap.get(DcMotor.class, "spinner_arm"); //spinner for big arm in intake
//        servo = hardwareMap.get(Servo.class, "servoTest");
        contServo =  hardwareMap.crservo.get("cont_Servo");
        servoFlip = hardwareMap.get(Servo.class, "flip_Intake");

        // Set Motor variables
        // Motors using encoders:
        // spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //spinnerArm.setTargetPosition(200);
        // set motors to run to target encoder position and stop with brakes on.
        //spinnerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // spinnerArm.setPower(1);

        // Motors without encoders
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        spinnerArm.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Define Variables
        // Setup a variable for each drive wheel to save power level for telemetry
        double LeftPower;
        double RightPower;
        double max;

        double rightPower;
        double servoposition;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // **** MOVE THE ARM ****
            if (gamepad2.a) {
                dropping = true;
                armTargetPos = Drop_Rotation;
            } else if (gamepad2.b) {
                dropping = false;
                armTargetPos = Pickup_Hover;
            }

            if (dropping) {
                Right_TrigPosition = armTargetPos + (int)(Drop_Range * gamepad2.left_trigger);
            } else {
                Right_TrigPosition = armTargetPos + 0;
            }

            /*  //Keep going down until touch sensor is triggered
            if (BoxTouch.isPressed()) {
                armPos = 0;
                spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                if (spinnerArm.getCurrentPosition() <= Pickup_Hover) {
                    armPos = armPos - 5;
                }
            }
            */

            CalcArmPower((int)Right_TrigPosition);

            spinnerArm.setPower(spinPower);


             // Move the Arm
            //spinnerArm.setTargetPosition((int) (armTargetPos+Right_TrigPosition));
            //spinnerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Missing code for Box Leveling servo







            // **** DRIVE THE ROBOT ****
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            LeftPower  = drive + turn;
            RightPower = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(LeftPower), Math.abs(RightPower));
            if (max > 1.0)
            {
                LeftPower /= max;
                RightPower /= max;
            }

            // Output the safe vales to the motor drives.
            leftDrive.setPower(LeftPower);
            rightDrive.setPower(RightPower);


            // Telemetry
            telemetry.addData("Move to:", armTargetPos+Right_TrigPosition);
            telemetry.addData("Curr Pos:",spinnerArm.getCurrentPosition());
            telemetry.addData("is at target", !spinnerArm.isBusy());

            telemetry.addData("Dropping?:", dropping);
            telemetry.update();



            // TO REVIEW

            servoposition = Range.clip(gamepad1.left_trigger, 0, 1);


            double intakePower = gamepad2.left_stick_y;

            // Set hex motor to game pad speed
            spinnerIntake.setPower(intakePower);

            // Set servo power to highest
            if (gamepad1.right_bumper) {
                contServo.setPower(1.0);
                } else {
                contServo.setPower(0);
            }


        }


            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", drivePower, turnPower);
            //telemetry.update();

            // If touch sensor hits the ground, set servo power to 0
            //if (touch.isPressed()) {
            //null;
            //}
        }

    private void CalcArmPower(int right_trigPosition) {
        // 1.Get current position
        // 2.Compare with target position
        // 3.Determine Side (Drop / PickUP)
        // 4.Determine Power need (High / Low)
        // 5.High Power => Power 100%
        // 6.Low Power => Power 30%
        // 7. Mid Power => Power 50%

        // Asumptions
        // Pickup is 0
        // Drop is 160
        //
        // Variables
        double OvershootPower = .3;
        double FullPower = 1;
        int MoveThreshold = 5;
        // 1. Get current position
        armPos = spinnerArm.getCurrentPosition();

        if (Math.abs(Right_TrigPosition - armPos) < MoveThreshold) { //Inside Threshold don't move
            spinPower = 0;
        } else {  // Need to move the arm
            if (armPos <= (Drop_Rotation/6*3)) {        //I'm in PickUp Zone
                if(armPos < Right_TrigPosition) { //I'm lower than target
                    spinPower = FullPower;
                } else {  // I'm higher than target
                    spinPower = -OvershootPower;
                }
            } else if (armPos >= (Drop_Rotation*5/6)) { //I'm in the Drop Zone
                if(armPos < Right_TrigPosition) { //I'm higher than target
                    spinPower = OvershootPower;
                } else {  // I'm higher than target
                    spinPower = -FullPower;
                }
            } else {                                    // I'm in the middle Zone
                if(armPos < Right_TrigPosition) { //I'm higher than target
                    spinPower = 0.5;
                } else {  // I'm higher than target
                    spinPower = -0.5;
                }

            }
        }

    }
}
