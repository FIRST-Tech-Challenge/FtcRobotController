package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Syborgs extends LinearOpMode {
    private DcMotor
            frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive;
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private DcMotor hangMotor;
    private CRServo intake;
    private Servo wrist;

    final double ARM_TICKS_PER_DEGREE = 28.0 * 250047.0 / 4913.0 * 100.0 / 20.0 / 360.0;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    final double WRIST_FOLDED_IN = 1.0 / 6.0;
    final double WRIST_FOLDED_OUT = 0.5;
    // slightly change the position if it isn't working
    final double FUDGE_FACTOR = 15.0 * ARM_TICKS_PER_DEGREE;
    double armPosition = Math.floor(ARM_COLLAPSED_INTO_ROBOT);
    double armPositionFudgeFactor;

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_COLLAPSED = 0.0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0.0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480.0 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;
    double cycleTime = 0, loopTime = 0, oldTime = 0, armLiftComp = 0;
    final double ARM_LIFT_COMPENSATION_FACTOR = 0.25568;

    @Override
    public void runOpMode() {
        double left, right, forward, rotate, max;
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        hangMotor = hardwareMap.dcMotor.get("hangMotor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake = hardwareMap.crservo.get("intake");
        wrist = hardwareMap.servo.get("wrist");

        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x; // forward back movement
            double y = -gamepad1.left_stick_y; // strafe movement
            double rotation = gamepad1.right_stick_x; // rotation
            if (gamepad1.options) {
                imu.resetYaw();
            }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotation), 1);
            double frontLeftPower = (rotY + rotX + rotation) / denominator;
            double frontRightPower = (rotY - rotX - rotation) / denominator;
            double backLeftPower = (rotY - rotX + rotation) / denominator;
            double backRightPower = (rotY + rotX - rotation) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.right_bumper) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad1.y) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            // Gamepad a sets the arm and wrist to intake/collecting position
            if (gamepad1.a) {
                armPosition = ARM_COLLECT;

                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);

                // Gamepad b sets position 20 degrees up to clear the barrier
                // doesn't move anything else
            } else if (gamepad1.b) {
                armPosition = ARM_CLEAR_BARRIER;

                // Gamepad x sets the arm to score the sample in the low basket
            } else if (gamepad1.x) {
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;

                // Turns off the intake (starting configuration)
                // Folds in wrist and collapses arm into robot
            } else if (gamepad1.dpad_left) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT;

                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);

                // Sets arm height to score specimen in high basket
            } else if (gamepad1.dpad_right) {
                armPosition = ARM_ATTACH_HANGING_HOOK;

                wrist.setPosition(WRIST_FOLDED_IN);

                // Sets the arm vertical on the low rung for hanging
            } else if (gamepad1.dpad_up) {
                armPosition = ARM_ATTACH_HANGING_HOOK;

                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);

                // Moves the arm down to lift the robot up once it has been hooked onto the low rung
            } else if (gamepad1.dpad_down) {
                armPosition = ARM_SCORE_SPECIMEN;

                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            // Compensating arm angle based on extension of arm
            if (armPosition < 45 * ARM_TICKS_PER_DEGREE) {
                armLiftComp = ARM_LIFT_COMPENSATION_FACTOR * liftPosition;
            } else {
                armLiftComp = 0;
            }
            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
            if (armMotor instanceof DcMotorEx) {
                ((DcMotorEx) armMotor).setVelocity(2100);
            }
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set lift position based on driver input
            if (gamepad2.right_bumper) {
                liftPosition += 2800 * cycleTime;
            } else if (gamepad2.left_bumper) {
                liftPosition -= 2800 * cycleTime;
            }
            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET) {
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            if (liftPosition < 0) {
                liftPosition = 0;
            }
            liftMotor.setTargetPosition((int) liftPosition);
            if (liftMotor instanceof DcMotorEx) {
                ((DcMotorEx) liftMotor).setVelocity(2100);
            }
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (armMotor instanceof DcMotorEx && ((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT");
            }
            hangMotor.setPower(-gamepad2.left_stick_y);

            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;

            telemetry.addData("Arm Target Position", armMotor.getTargetPosition());
            telemetry.addData("Arm Encoder", armMotor.getCurrentPosition());
            telemetry.addData("Lift Variable", liftPosition);
            telemetry.addData("Lift Target Position", liftMotor.getTargetPosition());
            telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
            telemetry.addData("Lift motor current", ((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
