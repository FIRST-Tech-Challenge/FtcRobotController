package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Disabled
@Autonomous(name = "RobotCenteringTest", group = "Test")
public class RobotCenteringTest extends ConeDetection
{
    int[] lowerBlue = {42, 128, 114};
    int[] upperBlue = {168, 242, 255};
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        waitForStart();
        driveSlides(400);
        driveGrabber(Constants.GRABBER_OPEN_POSITION);
        detectGrab(lowerBlue,upperBlue);
        while(opModeIsActive()){
            if(coneDetectionPipeline.distance > 10){
                telemetry.addData("distance", coneDetectionPipeline.distance);
                telemetry.update();
                motorFL.setPower(1*Math.signum(coneDetectionPipeline.distance));
                motorFR.setPower(-1*Math.signum(coneDetectionPipeline.distance));
                motorBL.setPower(-1*Math.signum(coneDetectionPipeline.distance));
                motorBR.setPower(1*Math.signum(coneDetectionPipeline.distance));
            }
        }
    }
}
