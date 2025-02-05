package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name="Field-Centric Teleop To Use :))))",group = "Teleops to use :))))))")
public class SecondRealTeleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    final int ASCENT_UP = 14200;
    final int ASCENT_DOWN = 2200;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //other motors
    DcMotor armLifterLeft = null;
    DcMotor armLifterRight = null;
    DcMotor armRotate = null;
    DcMotor linearActuator = null;
    //servos
    ServoImplEx wrist = null;
    Servo grabber = null;


    double actuatorPos = 0;
    double armRotPos = 0;
    double wristPos = 0.4;
    boolean canToggle = true;
    boolean open = true;
    final double ARMROTMULT = 60.0/43.0;

    void linearActuatorMover() {
        linearActuator.setTargetPosition((int) actuatorPos);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void controlGrabber(double i) {
        grabber.setPosition(i);
    }

    double armPos = 0;

    void controlBothArmExtenders() {
        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition(-(int) armPos);
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void runOpMode() {

        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(),new ArrayList<>());
        localizer.setPoseEstimate(new Pose2d());

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        armLifterLeft = hardwareMap.dcMotor.get("armLifterLeft");
        armLifterRight = hardwareMap.dcMotor.get("armLifterRight");
        armRotate = hardwareMap.dcMotor.get("armRotate");
        linearActuator = hardwareMap.dcMotor.get("linearActuator");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        wrist.setPwmRange(new PwmRange(500, 2500));
        grabber = hardwareMap.servo.get("grabber");

        armLifterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Timer timer = new Timer();
        long delay = 250L;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();
        runtime.reset();

        boolean isGrabbing = false;
        armLifterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLifterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLifterRight.setPower(1);
        armLifterLeft.setPower(1);
        linearActuator.setPower(1);
        armRotate.setPower(1);

        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition((int) armPos);
        linearActuator.setTargetPosition((int) actuatorPos);
        armRotate.setTargetPosition((int) armRotPos);
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            localizer.update();
            linearActuatorMover();
            controlBothArmExtenders();
            double pose = drive.getExternalHeading(); // used for my custom function to drive straight thru the trusses

            double slowMode = gamepad1.left_trigger;
            double slowCoeff = 0.3;

            double fastMode = gamepad1.right_trigger;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;


                double botHeading = drive.getPoseEstimate().getHeading();

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                if(slowMode > 0.2){
                    frontLeftMotor.setPower(frontLeftPower*slowCoeff);
                    frontRightMotor.setPower(frontLeftPower*slowCoeff);
                    backLeftMotor.setPower(frontLeftPower*slowCoeff);
                    frontRightMotor.setPower(frontLeftPower*slowCoeff);
                }
            }

            //arm rotate
            armRotate.setTargetPosition((int) armRotPos);
            double armRotChange = gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y * 75;
            if (armRotate.getCurrentPosition() + armRotChange > -2200 * ARMROTMULT) {
                armRotPos += armRotChange * ARMROTMULT;
            } else if (armRotate.getCurrentPosition() - armRotChange < -50 * ARMROTMULT) {
                armRotPos -= armRotChange * ARMROTMULT;
            }

            if (armRotate.getCurrentPosition() < -2200 * ARMROTMULT) {
                armRotate.setPower(0.25);
                armRotPos = -2100*ARMROTMULT;
            } else if(armRotate.getCurrentPosition() > -50 * ARMROTMULT){
                armRotate.setPower(0.25);
                armRotPos = -150*ARMROTMULT;
            } else {
                armRotate.setPower(1);
            }
            //actuator
            if (gamepad2.dpad_up) {
                actuatorPos = ASCENT_UP;
//                linearActuator.setPower(1);
            } else if (gamepad2.dpad_down) {
                actuatorPos = ASCENT_DOWN;
//                linearActuator.setPower(-1);
            }// else {
//                linearActuator.setPower(0);
//            }

            //extenders
            double rightTrig = gamepad2.right_trigger;
            double leftTrig = gamepad2.left_trigger;
            if (armPos - rightTrig * 75 > -5833) {
                armPos -= rightTrig * 75;
            }
            if (armPos + leftTrig * 75 < 2000) {
                armPos += leftTrig * 75;
            }
//            armLifterLeft.setPower(gamepad2.right_stick_y);
//            armLifterRight.setPower(gamepad2.right_stick_y);

//            //wrist
//            if(gamepad2.dpad_left){
//                wristPos = 0.5;}
            if (-gamepad2.right_stick_y > 0.01 && wristPos + 0.005 * 0.75 <= 1) {
                wristPos += 0.005;
            } else if (-gamepad2.right_stick_y < -0.01 && wristPos - 0.005 * 0.75 >= 0.4) {
                wristPos -= 0.005;
            }
            wrist.setPosition(wristPos);

            //grabber
            if (canToggle && gamepad2.x) {
                TimerTask task = new TimerTask() {
                    @Override
                    public void run() {
                        canToggle = true;
                    }
                };
                canToggle = false;
                open = !open;
                if(open){
                    controlGrabber(0.4);
                } else {
                    controlGrabber(0.75);
                }
                timer.schedule(task, delay);

            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pose", "Pose: " + localizer.getPoseEstimate());
            telemetry.addData("Arm height", "Arm height: " + armPos);
            telemetry.addData("Arm rot", "Arm rot: " + armRotPos);
            telemetry.addData("wrist rot", "Wrist rot: " + wristPos);
            telemetry.update();
        }
    }
    }