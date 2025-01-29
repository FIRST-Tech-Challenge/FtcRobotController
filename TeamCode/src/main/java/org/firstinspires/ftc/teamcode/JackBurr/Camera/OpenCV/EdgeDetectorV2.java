package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class EdgeDetectorV2 extends OpMode {
    public int cameraMonitorViewId;
    public OpenCvWebcam camera;
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public DifferentialV2 diff = new DifferentialV2();
    int width = 1280;
    int height = 800;
    int WIDTH_OLD_CAMERA = 1280;
    int HEIGHT_OLD_CAMERA = 960;
    public boolean isOldCamera = true;
    public EdgeDetectionPipeline pipeline = new EdgeDetectionPipeline();
    @Override
    public void init() {
        diff.init(hardwareMap, telemetry);
        diff.setTopLeftServoPosition(constants.FRONT_LEFT_HOVER);
        diff.setTopRightServoPosition(constants.FRONT_RIGHT_HOVER);
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        if(isOldCamera) {
            width = WIDTH_OLD_CAMERA;
            height = HEIGHT_OLD_CAMERA;
            pipeline.init(width, height, new MultipleTelemetry(telemetry, dashboard.getTelemetry()), true);
        }
        else {
            pipeline.init(width, height, new MultipleTelemetry(telemetry, dashboard.getTelemetry()), false);
        }
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                if(isOldCamera){
                    camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                }
                else {
                    camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_RIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                }
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });
    }

    @Override
    public void loop() {
        camera.stopStreaming();
    }
}
