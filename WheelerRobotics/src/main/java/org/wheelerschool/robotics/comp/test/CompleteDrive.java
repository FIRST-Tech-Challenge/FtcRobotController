package org.wheelerschool.robotics.comp.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.wheelerschool.robotics.comp.CompBot;

@TeleOp
public class CompleteDrive extends OpMode {
    CompBot hw;
    boolean launchMode = false;

    @Override
    public void init() {
        hw = new CompBot(hardwareMap);
    }

    @Override
    public void start() {
        hw.setWobbleArm(CompBot.WobblePosition.STOWED);
    }

    @Override
    public void loop() {
        float forward = gamepad1.left_stick_y;
        float strafe = gamepad1.left_stick_x;
        float rotate = gamepad1.right_stick_x;

        hw.setDrive(
                forward - rotate - strafe,
                forward + rotate + strafe,
                forward - rotate + strafe,
                forward + rotate - strafe
        );

        if (gamepad1.a) {
            launchMode = true;
        } else if (gamepad1.b) {
            launchMode = false;
        }

        hw.launcher(launchMode);

        hw.launchPush(gamepad1.right_bumper);

        if (gamepad1.left_bumper) {
            hw.intakeMode(CompBot.IntakeMode.IN);
        } else if (gamepad1.left_trigger > 0.2f) {
            hw.intakeMode(CompBot.IntakeMode.OUT);
        } else {
            hw.intakeMode(CompBot.IntakeMode.STOP);
        }

        hw.setWobbleGrab(gamepad1.y);


        if (gamepad1.dpad_up) {
            hw.setWobbleArm(CompBot.WobblePosition.UP);
        } else if (gamepad1.dpad_down) {
            hw.setWobbleArm(CompBot.WobblePosition.GRAB);
        }

        telemetry.addData("Arm current", hw.wobbleArm.getCurrentPosition());
        telemetry.addData("Arm target", hw.wobbleArm.getTargetPosition());
    }

    @Override
    public void stop() {
        hw.stop();
    }
}
