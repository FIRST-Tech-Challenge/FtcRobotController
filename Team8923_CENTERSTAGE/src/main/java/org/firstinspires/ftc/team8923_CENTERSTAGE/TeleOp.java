package org.firstinspires.ftc.team8923_CENTERSTAGE;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")

public class TeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        
        while (opModeIsActive()) {
            double y = (-gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            driveMecanum(x, y, pivot);

            idle();
        }
    }
}
