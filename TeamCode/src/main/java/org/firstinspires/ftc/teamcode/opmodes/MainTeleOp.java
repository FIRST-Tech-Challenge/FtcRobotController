package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        final float speedMultiplier = .40;
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                robot.driveTrain.setSpeedMultiplier(speedMultiplier);
            } else{
                robot.driveTrain.setSpeedMultiplier(1);
            }
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
        }
    }
}
