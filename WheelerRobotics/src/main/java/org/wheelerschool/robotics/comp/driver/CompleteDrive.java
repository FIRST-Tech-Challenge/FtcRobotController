package org.wheelerschool.robotics.comp.driver;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.wheelerschool.robotics.comp.CompBot;
import org.wheelerschool.robotics.lib.StatefulButton;

@TeleOp
public class CompleteDrive extends OpMode {
    CompBot hw;
    boolean launchMode = false;

    StatefulButton intakeCtl;
    StatefulButton wobbleCtl;
    boolean driveMode = true;

    @Override
    public void init() {
        hw = new CompBot(hardwareMap);

        intakeCtl = new StatefulButton(false);
        wobbleCtl = new StatefulButton(true);
    }

    @Override
    public void start() {
        hw.setWobbleArm(CompBot.WobblePosition.STOWED);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            driveMode = true;
        } else if (gamepad1.x) {
            driveMode = false;
        }

        float driveFactor = (driveMode)? 1 : -1;
        float forward = driveFactor * gamepad1.left_stick_y;
        float strafe = driveFactor * gamepad1.left_stick_x;
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

        intakeCtl.update(gamepad1.left_bumper);

        if (gamepad1.left_trigger > 0.2f) {
            intakeCtl.state = false;
            hw.intakeMode(CompBot.IntakeMode.OUT);
        } else if (intakeCtl.state) {
            hw.intakeMode(CompBot.IntakeMode.IN);
        } else {
            hw.intakeMode(CompBot.IntakeMode.STOP);
        }

        wobbleCtl.update(gamepad1.dpad_left);
        hw.setWobbleGrab(wobbleCtl.state);


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
