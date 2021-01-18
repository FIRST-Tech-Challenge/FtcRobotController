package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="VisionTest")
public class VisionTest extends LinearOpMode {

    private OpenCvCamera camera;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution


    private void initRobot() {
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        camera = OpenCvCameraFactory
                .getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();
        waitForStart();
//        double speed = 0.7;
//        driveLeft.set(-speed);
//        driveRight.set(speed);

        Thread.sleep(2000);

        stop();
    }
}

