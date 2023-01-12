package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;

@Disabled
@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends BaseTeleOp {

    double currentAngle;
    double headingDegrees;
    double negativeHeadingRadians;

    double x;
    double y;
    double t;

    double xRotatedVector;
    double yRotatedVector;;
    double ratio;

    double xPower;
    double yPower;
    double tPower;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            headingDegrees = currentAngle - startAngle;
            negativeHeadingRadians = Math.toRadians(-headingDegrees);

            if (gamepad1.left_trigger > 0.25) {
                x = gamepad1.left_stick_x * 0.3;
                y = -gamepad1.left_stick_y * 0.3;
                t = gamepad1.right_stick_x * 0.2;
            } else {
                x = gamepad1.left_stick_x * 0.75;
                y = -gamepad1.left_stick_y * 0.75;
                t = gamepad1.right_stick_x * 0.5;
            }

            xRotatedVector = x * Math.cos(negativeHeadingRadians) - y * Math.sin(negativeHeadingRadians);
            yRotatedVector = x * Math.sin(negativeHeadingRadians) + y * Math.cos(negativeHeadingRadians);

            ratio = 1 / Math.max(Math.abs(xRotatedVector) + Math.abs(yRotatedVector) + Math.abs(t), 1);

            xPower = xRotatedVector * ratio;
            yPower = yRotatedVector * ratio;
            tPower = t * ratio;

            driveWithIMU(xPower, yPower, tPower);

            telemetry.addData("start", startAngle);
            telemetry.addData("current", currentAngle);
            telemetry.addData("heading", headingDegrees);
            telemetry.update();
        }
    }
}
