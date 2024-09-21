package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.examples.TeleOpEnhancements;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    Chassis driveTrain;
    Arm arm;
    Intake intake;
    TeleOpEnhancements TPE;

    double y;
    double x;
    double rx;

    @Override
    public void init() {
        driveTrain = new Chassis(hardwareMap); // Initialize chassis
        arm = new Arm(hardwareMap);  // Initialize arm system
        intake = new Intake(hardwareMap);  // Initialize intake system

        TPE = new TeleOpEnhancements();
        TPE.init(); // Initialize TPE and its components

        // Verify TPE initialization
        if (TPE.follower != null) {
            TPE.follower.startTeleopDrive();
        } else {
            telemetry.addData("Error", "TPE Follower is not initialized.");
        }
    }

    @Override
    public void loop() {

        TPE.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        TPE.follower.update();

        // Drivetrain control
        y = applyResponseCurve(gamepad1.left_stick_y);
        x = -applyResponseCurve(gamepad1.left_stick_x);
        rx = -applyResponseCurve(gamepad1.right_stick_x);
        driveTrain.drive(x, y, rx);

        // Arm control using gamepad2
        if (gamepad2.dpad_up) {
            arm.setArmPosition(1000);  // target position
        } else if (gamepad2.dpad_down) {
            arm.setArmPosition(0);  // Lower position
        }
        
        // Rotation control using gamepad2
        if (gamepad2.left_bumper) {
            arm.rotateArm(0.5);  // Rotate arm clockwise
        } else if (gamepad2.right_bumper) {
            arm.rotateArm(-0.5);  // Rotate arm counterclockwise
        } else {
            arm.rotateArm(0);  // Stop rotation
        }

        // Intake control using gamepad2
        if (gamepad2.right_trigger > 0.1) {
            intake.setIntakePower(1.0);  // Activate intake
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setIntakePower(-1.0);  // Reverse intake
        } else {
            intake.setIntakePower(0);  // Stop intake
        }

        // Pivot control for intake
        if (gamepad2.dpad_left) {
            intake.setPivotPosition(0.0);  // Pivot intake to a certain position
        } else if (gamepad2.dpad_right) {
            intake.setPivotPosition(1.0);  // Pivot intake to another position
        }

        // Telemetry for diagnostics
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Arm Position", arm.atTargetPosition() ? "At Target" : "Moving");
        telemetry.addData("Servo Position", gamepad2.a ? "Open" : gamepad2.b ? "Closed" : "Neutral");
        telemetry.update();
    }

    // Response curve function for finer joystick control
    public double applyResponseCurve(double input) {
        double exponent = 2;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }
}
