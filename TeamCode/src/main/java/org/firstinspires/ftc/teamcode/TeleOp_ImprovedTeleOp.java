package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/** This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list. */

enum SpeedMode{
    SLOW, MED, FAST
}
@TeleOp(name="Improved TeleOP TeleOp", group="Iterative Opmode")

// @Disabled
public class TeleOp_ImprovedTeleOp extends OpMode
{
    /** Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor duckWheel = null;
    private DcMotor frontL = null;
    private DcMotor frontR = null;
    private DcMotor backL = null;
    private DcMotor backR = null;
    private CRServo intakeL = null;
    private CRServo intakeR = null;
    private DcMotor extender = null;
    private DcMotor arm = null;
    private DcMotor intakeLift = null;


    /** Code to run ONCE when the driver hits INIT. */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        /** Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone). */
        duckWheel = hardwareMap.get(DcMotor.class, "duckWheel");
        frontL  = hardwareMap.get(DcMotor.class, "leftFront");
        frontR = hardwareMap.get(DcMotor.class, "rightFront");
        backL  = hardwareMap.get(DcMotor.class, "leftRear");
        backR = hardwareMap.get(DcMotor.class, "rightRear");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        extender = hardwareMap.get(DcMotor.class, "extender");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");


        /* Sets the motors to run using encoders. */
        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* makes the motors break on zero power */
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /* Most robots need the motor on one side to be reversed to drive forward.
         * Reverse the motor that runs backwards when connected directly to the battery. */
        frontL.setDirection(DcMotor.Direction.FORWARD);
        backL.setDirection(DcMotor.Direction.FORWARD);
        frontR.setDirection(DcMotor.Direction.REVERSE);
        backR.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(CRServo.Direction.REVERSE);
        intakeR.setDirection(CRServo.Direction.FORWARD);
        duckWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLift.setDirection(DcMotorSimple.Direction.FORWARD);
      //  lIntakeLift.setDirection(CRServo.Direction.REVERSE);
        //rIntakeLift.setDirection(CRServo.Direction.FORWARD);




        /* Tell the driver that initialization is complete. */
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    /** Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY. */
    @Override
    public void init_loop() {

    }


    /** Code to run ONCE when the driver hits PLAY. */
    @Override
    public void start() {
        runtime.reset();
    }
    boolean intakeOn = false;
    boolean duckOn = false;
    double armPow = 0;
    double liftPow = 0;
    SpeedMode speed = SpeedMode.MED;
    double slowSpeed = 0.25;
    double normalSpeed = 0.69;
    double intakeLiftPow = 0;


    /** Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP. */
    @Override
    public void loop() {
        /* Setup a variable for each drive wheel to save power level for telemetry. */
        double leftFPower ;
        double rightFPower;
        double leftBPower ;
        double rightBPower;
        double intakePow;

        /* More variable setup*/
        double drive = -gamepad1.right_stick_y;
        double turn  =  gamepad1.left_stick_x * 0.3;
        double strafe = gamepad1.right_stick_x;
        double isIntakeA = gamepad2.left_trigger;
        double isIntakeB = gamepad2.right_trigger;
        boolean isDuckR = gamepad1.right_bumper;
        boolean isDuckL = gamepad1.left_bumper;
        double duckPower= 0;
        double extension = gamepad2.right_stick_y;
        double armMove = gamepad2.left_stick_y;
        boolean liftIntakeUp = gamepad2.dpad_up;
        boolean liftIntakeDown = gamepad2.dpad_down;

        if(gamepad1.dpad_up)
            speed = SpeedMode.FAST;
        if(gamepad1.dpad_down)
            speed = SpeedMode.SLOW;


        if (isIntakeA !=0) {
            intakePow = isIntakeA;
        }
        else if (isIntakeB !=0){
            intakePow = -1 * isIntakeB;
        }
        else intakePow = 0;


        if (isDuckR) {
            if (!duckOn){
                duckPower = 0.5;
                duckOn = true;

            }
            else if (duckOn) {
                duckPower = 0;
                duckOn = false;
            }
        }

        if (isDuckL) {
            if (!duckOn){
                duckPower = -0.5;
                duckOn = true;

            }
            else if (duckOn) {
                duckPower = 0;
                duckOn = false;
            }
        }

        if (liftIntakeUp) {
            intakeLiftPow = 0.75;
        }

        else if (liftIntakeDown) {
            intakeLiftPow = -0.75;
        }

        else{
            intakeLiftPow = 0;
        }

         if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        }

         else if (strafe != 0 ) {
            /* Strafing */
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower =  strafe;
            rightBPower = -strafe;
        }

        else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }





        if (speed == SpeedMode.FAST) {

            frontL.setPower(leftFPower);
            backL.setPower(leftBPower);
            frontR.setPower(rightFPower);
            backR.setPower(rightBPower);
            intakeR.setPower(intakePow);
            intakeL.setPower(intakePow);
            duckWheel.setPower(duckPower);
            extender.setPower(extension);
            arm.setPower(armMove * 1);
            intakeLift.setPower(intakeLiftPow);
        }

        else if (speed == SpeedMode.SLOW) {
            frontL.setPower(leftFPower * slowSpeed);
            backL.setPower(leftBPower * slowSpeed);
            frontR.setPower(rightFPower * slowSpeed);
            backR.setPower(rightBPower * slowSpeed);
            intakeR.setPower(intakePow);
            intakeL.setPower(intakePow);
            duckWheel.setPower(duckPower);
            extender.setPower(extension);
            arm.setPower(armMove * 1);
            intakeLift.setPower(intakeLiftPow);
        }
        else {
            frontL.setPower(leftFPower * normalSpeed);
            backL.setPower(leftBPower * normalSpeed);
            frontR.setPower(rightFPower * normalSpeed);
            backR.setPower(rightBPower * normalSpeed);
            intakeR.setPower(intakePow);
            intakeL.setPower(intakePow);
            duckWheel.setPower(duckPower);
            extender.setPower(extension);
            arm.setPower(armMove * 1);
            intakeLift.setPower(intakeLiftPow);
        }


        /**  Show the elapsed game time and wheel power. */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", leftFPower, rightFPower,leftBPower, rightBPower);
        telemetry.addData("Intake Power", intakePow );
        telemetry.addData("Arm Power","power %d encoder %d",arm.getPowerFloat(), arm.getCurrentPosition());
        telemetry.addData("Extension Power", extender.getPowerFloat());
        telemetry.addData("Intake lift","power %d encoder %d", intakeLift.getPowerFloat(), intakeLift.getCurrentPosition());


    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop() {

    }
}
