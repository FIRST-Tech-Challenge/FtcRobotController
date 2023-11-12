package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Tele", group = "Pushbot")
public class tele extends OpMode {
    Hardware robot = new Hardware();

    Gamepad.LedEffect gamepadLaunchSequenceLed = new Gamepad.LedEffect.Builder()
            .addStep(100, 0, 0, 2000)
            .addStep(0,100,0,200)
            .addStep(100,0,0,200)
            .addStep(0,100,0,200)
            .addStep(100,0,0,200)
            .addStep(0,100,0,200)
            .addStep(100,0,0,200)
            .addStep(0,100,0,200)
            .addStep(100,0,0,200)
            .addStep(0,100,0,200)
            .addStep(100,0,0,200)
            .addStep(0,100,0,200)
            .addStep(100,0,0,200)
            .addStep(0,100,0,200)
            .addStep(100,0,0,200)
            .addStep(100,0,100,5000)
            .build();
    Gamepad.RumbleEffect  gamepadLaunchSequenceRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(.3,.3,2000)
            .addStep(.5,0,200)
            .addStep(0,.5,200)
            .addStep(.5,0,200)
            .addStep(0,.5,200)
            .addStep(.5,0,200)
            .addStep(0,.5,200)
            .addStep(.5,0,200)
            .addStep(0,.5,200)
            .addStep(.5,0,200)
            .addStep(0,.5,200)
            .addStep(.5,0,200)
            .addStep(0,.5,200)
            .addStep(.5,0,200)
            .addStep(0,.5,200)
            .build();



    public void init() {

        robot.init(hardwareMap);


    }

    public void start() {

    }

    public void loop() {
        double speedLimit;

        if (gamepad1.right_trigger > .5) {
            speedLimit = 50;
        } else {
            speedLimit = 90;
        }

        double speedLimitValue = speedLimit/100;

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.leftDrive.setPower(frontLeftPower * speedLimitValue);
        robot.leftBackDrive.setPower(backLeftPower * speedLimitValue);
        robot.rightDrive.setPower(frontRightPower * speedLimitValue);
        robot.rightBackDrive.setPower(backRightPower * speedLimitValue);

        robot.winch.setPower(-gamepad2.right_stick_y);



        if (gamepad2.dpad_up) {
            robot.hook.setPosition(1.0);
        } else if (gamepad2.dpad_down) {
            robot.hook.setPosition(0);
        }

        if (gamepad2.y) {
            robot.droneAngle.setPosition(.32);
        } else if (gamepad2.a) {
            robot.droneAngle.setPosition(.17);
        }

        if (gamepad2.x && gamepad2.b) {
            robot.launcherRelease.setPosition(0.85);
        } else {
            robot.launcherRelease.setPosition(0.6);
        }

        if (gamepad2.right_bumper) {
            robot.arm.setPosition(.4);
        } else if (gamepad2.left_bumper) {
            robot.arm.setPosition(0);
        }

        if (gamepad2.right_trigger > 0.5) {
            robot.gripper.setPosition(0.55);
        } else if (gamepad2.left_trigger > 0.5) {
            robot.gripper.setPosition(1);
        }

        if (gamepad2.right_bumper && gamepad1.left_bumper) {
            gamepad1.runLedEffect(gamepadLaunchSequenceLed);
            gamepad2.runLedEffect(gamepadLaunchSequenceLed);
            gamepad1.runRumbleEffect(gamepadLaunchSequenceRumble);
            gamepad2.runRumbleEffect(gamepadLaunchSequenceRumble);
        }

        telemetry.addData("Right Distance", robot.rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", robot.leftDistance.getDistance(DistanceUnit.CM));
        telemetry.update();

    }

}
