package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drive 24-25", group="Iterative Opmode")
public class TeleOpMode extends OpMode
{
    //DONT DELETE
    private ElapsedTime runtime = new ElapsedTime();
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    private double Kg = 0.07;
    private boolean boxUp = false;
    //private PIDController pid = new PIDController(0.06, 0, 0);


    //main thingies
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private DcMotor slideIntake = null;

    // lift encoder positions
    private int liftMotor1StartPosition;
    private int liftMotor1EndPosition;
    private int slideIntakeStartPosition; //109
    private int slideIntakeEndPosition;

    private long bumperPressTime = 0;
    private long hangTimeStart = 0;

    private Servo rotateArm = null;
    private Servo bottomClaw = null;
    private Servo rotateBClaw = null;
    private Servo flipTClaw = null;
    private Servo rotateTClaw = null;
    private Servo topClaw = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //wheels
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        rearLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        rearRight = hardwareMap.get(DcMotor.class, "rightBack");

        //lift
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        //things attached to lift
        slideIntake = hardwareMap.get(DcMotor.class,"slideIntake");
        rotateArm = hardwareMap.get(Servo.class, "rotateArm");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        rotateBClaw = hardwareMap.get(Servo.class, "rotateBClaw");
        flipTClaw = hardwareMap.get(Servo.class, "flipTClaw");
        rotateTClaw = hardwareMap.get(Servo.class, "rotateTClaw");
        topClaw = hardwareMap.get(Servo.class, "topClaw");


        //SETTINGS FOR MOTORS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //RUN_TO_POSITION

        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideIntakeStartPosition = slideIntake.getCurrentPosition();
        slideIntakeEndPosition = slideIntakeStartPosition - 1210;

        //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        //door.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        if (liftMotor1.getCurrentPosition() > 0) {
            liftMotor1StartPosition = 0;
        } else {
            liftMotor1StartPosition = liftMotor1.getCurrentPosition();
        }
        liftMotor1EndPosition = liftMotor1StartPosition + 4650;

        telemetry.addData("init-lift-start", liftMotor1StartPosition);
        telemetry.addData("lift1 current Position", liftMotor1.getCurrentPosition());
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //GAMEPAD 1 CONTROLS - Primary: Driving, Safety: Lift(Motors)
        //START DRIVING
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;

        double y = gamepad1.left_stick_y * -1;
        double x = gamepad1.left_stick_x * 1.5;
        double pivot = gamepad1.right_stick_x;

        frontLeftPower = (pivot+y+x);
        rearLeftPower = (pivot+y-x);
        frontRightPower = (-pivot+y-x);
        rearRightPower = (-pivot+y+x);

        if (gamepad1.dpad_down) {
            liftMotor1.setPower(-0.4);
            liftMotor2.setPower(-0.4);
        }

        if (gamepad1.dpad_left) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Driver control robot movements
        if(gamepad1.left_bumper) {
            // crawl mode
            frontLeft.setPower(frontLeftPower * 0.25);
            frontRight.setPower(frontRightPower * 0.25);
            rearLeft.setPower(rearLeftPower * 0.25);
            rearRight.setPower(rearRightPower * 0.25);
        }
        else {
            frontLeft.setPower(frontLeftPower * .9);
            frontRight.setPower(frontRightPower * .9);
            rearLeft.setPower(rearLeftPower * .9);
            rearRight.setPower(rearRightPower * .9);
        }

        // lift control, holding power and limits
        int liftMotor1Position = liftMotor1.getCurrentPosition();
        int slideIntakePosition = slideIntake.getCurrentPosition();

        if (gamepad2.right_stick_y < -0.8 && liftMotor1Position < (liftMotor1EndPosition)) { //if stick is forward
            liftMotor1.setPower(1.0);
            liftMotor2.setPower(1.0);
        } else if (gamepad2.right_stick_y > 0.8 && liftMotor1Position > (liftMotor1StartPosition)) { //if stick is back
            liftMotor1.setPower(-1.0);
            liftMotor2.setPower(-1.0);
        } else {
            liftMotor1.setPower(0.05);
            liftMotor2.setPower(0.05);
        }
        telemetry.addData("tele-lift-pos", liftMotor1.getCurrentPosition());
        telemetry.addData("tele-lift-power", liftMotor1.getPower());
        telemetry.addData("tele-lift-start", liftMotor1StartPosition);
        telemetry.addData("tele-lift-end", liftMotor1EndPosition);


        telemetry.addData("tele-hslide-pos", slideIntake.getCurrentPosition());

        // Driver - Bottom claw
        if (gamepad1.b) {
            bottomClaw.setPosition(0.2); // open bottom claw
        } else if (gamepad1.a) {
            bottomClaw.setPosition(1); // close bottom claw
        }

        if (gamepad1.right_trigger > 0.5) {
            hangTimeStart = System.nanoTime();
        }

        while (System.nanoTime() - hangTimeStart < 2100000000) {
            if (System.nanoTime() - hangTimeStart < 1000000000) {
                liftMotor1.setPower(-0.8);
                liftMotor2.setPower(-0.8);
            } else if (System.nanoTime() - hangTimeStart < 2100000000) {
                liftMotor1.setPower(-0.5);
                liftMotor2.setPower(-0.5);
            }
        }

        // Co-Driver Top claw
        if (gamepad2.b) {
            topClaw.setPosition(0.4); // open top claw
        } else if (gamepad2.a) {
            topClaw.setPosition(1); // close top claw
        }

        // Co-Driver Bottom claw rotation
        if (gamepad2.x) {
            rotateBClaw.setPosition(0.37); // rotate bottom claw to original position
        } else if (gamepad2.y) {
            rotateBClaw.setPosition(0.70); // rotate bottom claw to second position
        }

        if (gamepad2.dpad_down) {
            rotateArm.setPosition(1); // rotate bottom claw arm out
        }

        // Co-Driver Extend claw intake
        if (gamepad2.left_stick_y < -0.8 && slideIntakePosition > (slideIntakeEndPosition)) {
            slideIntake.setPower(-0.5); // slide out
        } else if (gamepad2.left_stick_y > 0.8  && slideIntakePosition < (slideIntakeStartPosition)) {
            slideIntake.setPower(0.5); // slide in
        } else {
            slideIntake.setPower(0);
        }

        // Co-Driver Top Claw
        if (gamepad2.dpad_up) {
            bottomClaw.setPosition(0.2); // open bottom claw
        }

        if (gamepad2.left_bumper) {
            flipTClaw.setPosition(0.98); // flip the top claw into the robot NEW
            rotateTClaw.setPosition(0.65); // rotate top claw to be vertical
            topClaw.setPosition(0.45); // open top claw
        }

        // Co-Driver Bottom Claw
        if (gamepad2.right_bumper) {
            // preset of bottom claw open, top claw close, lift down, top claw flip for exchange,
            topClaw.setPosition(1); // close top claw
            bumperPressTime = System.nanoTime();
            telemetry.addData("nanotimedif", System.nanoTime() - bumperPressTime);
        }

        if (bumperPressTime != 0 && System.nanoTime() - bumperPressTime > 300000000) {
            bottomClaw.setPosition(0.2); // open bottom claw
        }

        if (bumperPressTime != 0 && System.nanoTime() - bumperPressTime > 600000000) {
            flipTClaw.setPosition(0); // flip the top claw out of the robot
            bumperPressTime = 0;
        }



        /*if (gamepad1.x) {
            flipTClaw.setPosition(0);
        } */

        if (gamepad2.dpad_left) {
            //Rotate Bottom Claw, Bottom Close Claw, Retract, Flip Bottom Claw,
            rotateBClaw.setPosition(0.37); // rotate bottom claw to OG position
            bottomClaw.setPosition(1); // close bottom claw
            rotateArm.setPosition(0); // rotate bottom claw arm back in
            if (slideIntakePosition < (slideIntakeStartPosition - 300))  {
                slideIntake.setPower(0.7); // slide in
            }
        }


        if (gamepad2.dpad_right) {
            bottomClaw.setPosition(1); // close bottom claw
        }

        if (gamepad2.left_trigger > 0.2) {
            // top claw rotate, top claw open
            topClaw.setPosition(0.4); // open top claw
            flipTClaw.setPosition(0.2); // flip to align
            rotateTClaw.setPosition(0.88); // 0 is going up, 1 is going down
        }

        if (gamepad2.right_trigger > 0.3) {
            flipTClaw.setPosition(0.47);
            rotateTClaw.setPosition(1);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "frontLeft (%.2f), rearLeft (%.2f), frontRight (%.2f), rearRight (%.2f)", frontLeftPower, rearLeftPower, frontRightPower, rearRightPower);
        telemetry.addData("Slide:", "Power" + slideIntake.getPower());
        //telemetry.addData("Lift position", liftMotor1.getCurrentPosition());
        //telemetry.addData("Servo1", rightLiftServo.getPosition());
        //telemetry.addData("Servo2", leftLiftServo.getPosition());

        telemetry.update();
    }


    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

