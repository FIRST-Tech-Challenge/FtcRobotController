package org.firstinspires.ftc.team8923_CENTERSTAGE;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")

public class TeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        
        while (opModeIsActive()) {
            double y = (-gamepad1.left_stick_y)/10;
            double x = gamepad1.left_stick_x/10;
            double pivot = gamepad1.right_stick_x/10;

            double power = calculateDistance(x, y);

            double angle = Math.toDegrees(Math.atan2(y, x)); // 0 degrees is forward

            driveMecanum(x, y, pivot);

            // teleopDriving();

            idle();
        }
    }
}
