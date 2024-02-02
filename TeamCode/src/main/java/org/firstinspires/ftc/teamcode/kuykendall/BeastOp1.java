package org.firstinspires.ftc.teamcode.kuykendall;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.shared.MotionHardwareRuiming;

@TeleOp(name="BeastOp1", group="Test")
public class BeastOp1 extends LinearOpMode {
    private MotionHardwareRuiming robot = new MotionHardwareRuiming();
    private ElapsedTime runtime = new ElapsedTime();

    private final double pickupPosition = .65;
    private final double dropoffPosition = .2;
    private static final int PICKUP_POSITION = 0;
    private static final int DROPOFF_POSITION = 50;
    private final double outSlide = .37;
    private final double inSlide = .82;

    private boolean slowmoActive = false;
    private boolean slowmoToggle = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Initialization code that was in the init() method
        robot.wristServo.setPosition(0.8);
        robot.rightInt.setPosition(pickupPosition);
        robot.leftInt.setPosition(1.0 - pickupPosition);

        waitForStart();  // Wait for the start button to be pressed

        // Main operation loop
        while (opModeIsActive()) {
            // Code that was in the loop() method
            controlDrive();
            controlArm();
            controlServos();
            updateTelemetry();
        }
    }

    private void controlDrive() {
        double strafe = -gamepad2.left_stick_x;
        double rotate = -gamepad2.right_stick_x;
        double drive = gamepad2.left_stick_y;

        if (gamepad2.left_stick_button && !slowmoToggle) {
            slowmoActive = !slowmoActive;
            slowmoToggle = true;
        } else if (!gamepad2.left_stick_button) {
            slowmoToggle = false;
        }

        double speedModifier = slowmoActive ? 0.25 : 1.0;
        drive *= speedModifier;
        strafe *= speedModifier;
        rotate *= speedModifier;

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.backRightMotor.setPower(backRightPower);
    }

    private void controlArm() {
        if (gamepad1.a) {
            robot.setArmPosition(1.0, PICKUP_POSITION, 5.0, runtime, telemetry, this::opModeIsActive);
        } else if (gamepad1.y) {
            robot.setArmPosition(1.0, DROPOFF_POSITION, 5.0, runtime, telemetry, this::opModeIsActive);
        }

        if (gamepad1.dpad_left) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void controlServos() {
        if (gamepad1.right_bumper) {
            robot.bucketServo.setPosition(inSlide);
        } else if (gamepad1.left_bumper) {
            robot.bucketServo.setPosition(outSlide);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Wrist Servo Position", robot.wristServo.getPosition());
        telemetry.addData("Right Intake Servo Position", robot.rightInt.getPosition());
        telemetry.addData("Left Intake Servo Position", robot.leftInt.getPosition());
        telemetry.update();
    }
}
