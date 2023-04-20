package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
@Autonomous(name = "ConeCenteringTest")
public class ConeCenteringTest extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        robotCameraPipeline.invertRange(true);
        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);

        waitForStart();
        while(opModeIsActive()){

        }
    }
}
