package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TacoTele2")
public class TacoTele2 extends teleinit {

    @Override
    public void run() {
        waitForStart();
        if (opModeIsActive()) {
            lift.armRTP(1);
            while (opModeIsActive()) {
                drive.driving(gamepad1.right_stick_y,gamepad1.left_stick_y,gamepad1.a);
                intake.intaking(gamepad1.right_trigger);
                lift.armState(gamepad1.y, gamepad1.right_trigger, gamepad1.x, gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down);
                telemetry.addData("Lift", lift.winch.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
