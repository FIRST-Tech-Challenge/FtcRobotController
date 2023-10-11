package org.firstinspires.ftc.team8923_CENTERSTAGE;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")

abstract public class TeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        double x;
        double y;
        double rotationalPower;
        while (opModeIsActive()) {
            teleopDriving();

            idle();
        }
    }
}
