package org.firstinspires.ftc.team8923_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestTeleOp")

public class TestTeleOP extends BaseTeleOp {

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        waitForStart();

        referenceAngle = imu.getAngularOrientation().firstAngle;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotationalPower = gamepad1.right_stick_x;

            if (rotationalPower != 0) {
                referenceAngle = imu.getAngularOrientation().firstAngle;
            }

            double power = calculateDistance(x, y);

            double angle = Math.toDegrees(Math.atan2(y, x)); // 0 degrees is forward

            driveMecanumGyro(angle, power, rotationalPower);

            driveRobotSpeed();
            driveMechanism();
            driveClaw();

            idle();
        }
    }
}
