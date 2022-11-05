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
        waitForStart();

        telemetry.addLine("TO PREVENT START ROTATE WAIT A FEW SECONDS IN BETWEEN INIT AND START");

        driveTurntableWithController();

        // get angle after startup to prevent jitter on startup
        IMUOriginalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (opModeIsActive()) {
            driveChassisWithController();
            driveGrabberWithController();
            driveSlidesWithController();
            idle();
        }
    }
}
