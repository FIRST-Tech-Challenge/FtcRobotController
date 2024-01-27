package org.firstinspires.ftc.teamcode.vision.April;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "APRILTAGS")
public class AprilTagDemo extends CommandOpMode {
    OpenCvCamera camera;
    private AprilTagDetector detector;
    @Override
    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "coolio"), cameraMonitorViewId);
        detector = new AprilTagDetector(hardwareMap, "coolio", 432, 240,  0.166, 578.272, 578.272, 402.145, 221.506);

        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                 @Override
                 public void onOpened() {
                     camera.startStreaming(432,240, OpenCvCameraRotation.UPRIGHT);
                 }

                 @Override
                 public void onError(int errorCode) {

                 }
             });
        camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);

//        AprilTagDetector.Placement detections = detector.getPlacement();

        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
            telemetry.addData("Placement", detector.getPlacement());
            telemetry.update();
        })));
    }
}
