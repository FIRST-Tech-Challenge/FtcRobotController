package org.firstinspires.ftc.team6220_PowerPlay.Competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;

@TeleOp(name = "TeleOpCompetition", group = "Competition")
public class TeleOpCompetition extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        driveTurntableWithController();

        // get angle after startup to prevent jitter on startup
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive()) {
            driveChassisWithController();
            driveGrabberWithController();
            driveSlidesWithController();
            idle();
        }
    }
}
