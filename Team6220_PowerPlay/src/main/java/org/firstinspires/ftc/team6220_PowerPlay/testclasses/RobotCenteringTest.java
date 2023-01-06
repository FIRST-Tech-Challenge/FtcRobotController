package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RobotCenteringTest", group = "Test")
public class RobotCenteringTest extends ConeDetection
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        waitForStart();
        telemetry.addLine("waiting for start");
        telemetry.update();
        while(opModeIsActive()){
            if(coneDetectionPipeline.distance > 10){
                motorFL.setPower(1*Math.signum(coneDetectionPipeline.distance));
                motorFR.setPower(-1*Math.signum(coneDetectionPipeline.distance));
                motorBL.setPower(-1*Math.signum(coneDetectionPipeline.distance));
                motorBR.setPower(1*Math.signum(coneDetectionPipeline.distance));
            }
        }
    }
}
