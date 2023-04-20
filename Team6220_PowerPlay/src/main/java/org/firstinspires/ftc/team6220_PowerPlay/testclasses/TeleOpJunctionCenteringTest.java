package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@TeleOp(name = "TeleOpJunctionCenteringTest")
public class TeleOpJunctionCenteringTest extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();

        robotCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);
        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        robotCamera.setPipeline(robotCameraPipeline);
        waitForStart();

        while (opModeIsActive()) {
            driveFieldCentric();
            driveGrabberWithController();
            driveSlidesWithController();
            teleOpJunctionCentering();
            driveLEDs();
            resetIMU();
            telemetry.addData("stack height", motorLeftSlides.getCurrentPosition());
            telemetry.update();
        }
    }
}
