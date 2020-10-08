package TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TestOpModes.KashviMasterOpMode;

@TeleOp(name = "KashviTestTeleOp")


public class KashviTestTeleOp extends KashviMasterOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData ("init", "done");

        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            double turningPower = gamepad1.left_stick_x;
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x);
            double power = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

            moveMecanum(angle, power, turningPower);


        }
    }
}
