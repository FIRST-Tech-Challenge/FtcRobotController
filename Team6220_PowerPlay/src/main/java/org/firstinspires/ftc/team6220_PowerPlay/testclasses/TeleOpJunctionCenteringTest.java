package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;


@TeleOp(name = "TeleOpJunctionCenteringTest")
public class TeleOpJunctionCenteringTest extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();

        robotCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);
        
        waitForStart();

        while (opModeIsActive()) {
            driveFieldCentric();
            driveGrabberWithController();
            driveSlidesWithController();
            teleOpJunctionCentering();
            resetIMU();
        }
    }
}
