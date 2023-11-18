package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.DriveMethods;

public class SpeedRunTeleOp extends DriveMethods {
    @Override
    public void runOpMode() {
        initMotorsSecondBot();

        double leftX = gamepad1.left_stick_x;
        double leftY = -gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;
        double speedDiv = 2.0;
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            leftX = gamepad1.left_stick_x;
            leftY = -gamepad1.left_stick_y;
            rightX = gamepad1.right_stick_x;

            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                setMotorPowers(0.0, 0.0, 0.0, 0.0);
            } else {
                setMotorPowers(
                        -(leftY - leftX - rightX) / speedDiv,
                        -(leftY + leftX - rightX) / speedDiv,
                        (leftY + leftX + rightX) / speedDiv,
                        (leftY - leftX + rightX) / speedDiv
                );
            }
        }
    }
}
