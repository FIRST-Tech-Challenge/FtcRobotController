package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Bot_Tester", group = "Test")
public class Bot_Tester extends OpMode {

    private static final double STICK_DEADBAND = 0.05;
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final double SHOOTER_TARGET_RPM = 3100.0;
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private DcMotor frontIntakeMotor;
    private CRServo frontLeftServo;
    private CRServo frontRightServo;
    private CRServo windmillServo;

    private DcMotorEx shooterMotor;

    @Override
    public void init() {
        // Drive motor names kept identical to scrimmage master.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Use the same direction profile currently used by main TeleOp.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake and feeder hardware names kept identical to scoring controllers.
        frontIntakeMotor = hardwareMap.get(DcMotor.class, "front_intake_motor");
        frontLeftServo = hardwareMap.get(CRServo.class, "front_left_servo");
        frontRightServo = hardwareMap.get(CRServo.class, "front_right_servo");
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");

        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stopAllOutputs();
        telemetry.addLine("Bot_Tester initialized");
    }

    @Override
    public void loop() {
        runDriveMotorTests();
        runIntakeTest();
        runShooterAndWindmillTest();
        addTelemetry();
    }

    @Override
    public void stop() {
        stopAllOutputs();
    }

    private void runDriveMotorTests() {
        double leftY = -gamepad1.left_stick_y;
        double rightY = -gamepad1.right_stick_y;

        leftY = Math.abs(leftY) > STICK_DEADBAND ? leftY : 0.0;
        rightY = Math.abs(rightY) > STICK_DEADBAND ? rightY : 0.0;

        // Default: left stick Y controls left front, right stick Y controls right front.
        // Hold left bumper to switch left stick Y to left rear.
        // Hold right bumper to switch right stick Y to right rear.
        double frontLeftPower = 0.0;
        double backLeftPower = 0.0;
        double frontRightPower = 0.0;
        double backRightPower = 0.0;

        if (gamepad1.left_bumper) {
            backLeftPower = leftY;
        } else {
            frontLeftPower = leftY;
        }

        if (gamepad1.right_bumper) {
            backRightPower = rightY;
        } else {
            frontRightPower = rightY;
        }

        frontLeftDrive.setPower(Range.clip(frontLeftPower, -1.0, 1.0));
        frontRightDrive.setPower(Range.clip(frontRightPower, -1.0, 1.0));
        backLeftDrive.setPower(Range.clip(backLeftPower, -1.0, 1.0));
        backRightDrive.setPower(Range.clip(backRightPower, -1.0, 1.0));
    }

    private void runIntakeTest() {
        double intakeCommand = gamepad1.left_trigger;
        if (intakeCommand < TRIGGER_DEADBAND) {
            intakeCommand = 0.0;
        }

        // Intake trigger: both feeder servos + intake motor.
        frontIntakeMotor.setPower(intakeCommand);
        frontLeftServo.setPower(intakeCommand);
        frontRightServo.setPower(-intakeCommand);
    }

    private void runShooterAndWindmillTest() {
        double shooterCommand = gamepad1.right_trigger;
        if (shooterCommand < TRIGGER_DEADBAND) {
            shooterCommand = 0.0;
        }

        if (shooterCommand > 0.0) {
            double targetTicksPerSecond = (SHOOTER_TARGET_RPM / 60.0) * SHOOTER_TICKS_PER_REV;
            shooterMotor.setVelocity(targetTicksPerSecond);
            windmillServo.setPower(-1.0);
        } else {
            shooterMotor.setPower(0.0);
            windmillServo.setPower(0.0);
        }
    }

    private void stopAllOutputs() {
        if (frontLeftDrive != null) {
            frontLeftDrive.setPower(0.0);
        }
        if (frontRightDrive != null) {
            frontRightDrive.setPower(0.0);
        }
        if (backLeftDrive != null) {
            backLeftDrive.setPower(0.0);
        }
        if (backRightDrive != null) {
            backRightDrive.setPower(0.0);
        }
        if (frontIntakeMotor != null) {
            frontIntakeMotor.setPower(0.0);
        }
        if (frontLeftServo != null) {
            frontLeftServo.setPower(0.0);
        }
        if (frontRightServo != null) {
            frontRightServo.setPower(0.0);
        }
        if (windmillServo != null) {
            windmillServo.setPower(0.0);
        }
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }

    private void addTelemetry() {
        telemetry.addLine("=== Bot_Tester Controls (Gamepad1) ===");
        telemetry.addLine("Left stick Y: left front motor");
        telemetry.addLine("Left bumper + left stick Y: left rear motor");
        telemetry.addLine("Right stick Y: right front motor");
        telemetry.addLine("Right bumper + right stick Y: right rear motor");
        telemetry.addLine("Left trigger: intake motor + feeder servos");
        telemetry.addLine("Right trigger: shooter 3100 RPM + windmill");

        telemetry.addData("LF / RF", "%.2f / %.2f", frontLeftDrive.getPower(), frontRightDrive.getPower());
        telemetry.addData("LB / RB", "%.2f / %.2f", backLeftDrive.getPower(), backRightDrive.getPower());
        telemetry.addData("Intake trig", "%.2f", gamepad1.left_trigger);
        telemetry.addData("Shooter trig", "%.2f", gamepad1.right_trigger);
        telemetry.addData("Shooter vel tps", "%.1f", shooterMotor.getVelocity());
        telemetry.update();
    }
}

