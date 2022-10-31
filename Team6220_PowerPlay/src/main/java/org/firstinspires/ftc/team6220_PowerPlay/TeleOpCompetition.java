package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "TeleOpCompetition", group = "Competition")
public class TeleOpCompetition extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();

        telemetry.addLine("START A FEW SECONDS AFTER INITIALIZING TO PREVENT START ROTATING");
        telemetry.update();

        waitForStart();

        // get angle after startup to prevent jitter on startup
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        openGrabber(true);

        while (opModeIsActive()) {
            teleOpDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            idle();
        }
    }
}
