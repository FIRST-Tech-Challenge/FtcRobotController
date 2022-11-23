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

    @Override
    public void runOpMode() {
        double p = 10.0, i = 3.0, d = 0.0, f = 0.0;

        initialize();
        waitForStart();

        // get angle after startup to prevent jitter on startup
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive()) {
            driveChassisWithController();

            if (gamepad1.a) {
                if (gamepad1.dpad_up) {
                    p++;
                } else if (gamepad1.dpad_down) {
                    p--;
                }

            } else if (gamepad1.x) {
                if (gamepad1.dpad_up) {
                    i++;
                } else if (gamepad1.dpad_down) {
                    i--;
                }

            } else if (gamepad1.y) {
                if (gamepad1.dpad_up) {
                    d++;
                } else if (gamepad1.dpad_down) {
                    d--;
                }

            } else if (gamepad1.b) {
                if (gamepad1.dpad_up) {
                    f++;
                } else if (gamepad1.dpad_down) {
                    f--;
                }
            }

            motorFL.setVelocityPIDFCoefficients(p, i, d, f);
            motorFR.setVelocityPIDFCoefficients(p, i, d, f);
            motorBL.setVelocityPIDFCoefficients(p, i, d, f);
            motorBR.setVelocityPIDFCoefficients(p, i, d, f);

            telemetry.addData("p", p);
            telemetry.addData("i", i);
            telemetry.addData("d", d);
            telemetry.addData("f", f);
            telemetry.update();
        }
    }
}
