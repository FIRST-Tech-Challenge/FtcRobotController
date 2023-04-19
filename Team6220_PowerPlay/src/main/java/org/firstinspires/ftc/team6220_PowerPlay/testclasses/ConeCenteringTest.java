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

        robotCameraPipeline.setRanges(Constants.BLUE_SCALAR_ARRAY[0], Constants.BLUE_SCALAR_ARRAY[1]);

        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);

        waitForStart();
        centerConeStack(robotCameraPipeline,450,1);
    }
}
