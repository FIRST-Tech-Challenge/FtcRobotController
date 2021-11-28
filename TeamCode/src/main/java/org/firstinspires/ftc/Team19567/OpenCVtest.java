package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.Team19567.tsePipeline.LOCATION;

@Autonomous(name="OpenCV Test",group="Linear Opmode")
public class OpenCVtest extends LinearOpMode {
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        tsePipeline pipeline = new tsePipeline(telemetry);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("OpenCV","OpenCV actually connected wow");
                telemetry.update();

                waitForStart();
                switch(pipeline.getLocation()) {
                    case ALLIANCE_FIRST: {
                        location = tsePipeline.LOCATION.ALLIANCE_FIRST;
                        telemetry.addData("OpenCV","First Level Detected");
                        telemetry.update();
                    }
                    case ALLIANCE_SECOND: {
                        location = tsePipeline.LOCATION.ALLIANCE_SECOND;
                        telemetry.addData("OpenCV","Second Level Detected");
                        telemetry.update();
                    }
                    case ALLIANCE_THIRD: {
                        location = tsePipeline.LOCATION.ALLIANCE_THIRD;
                        telemetry.addData("OpenCV","Third Level Detected");
                        telemetry.update();
                    }
                    default: {
                        location = tsePipeline.LOCATION.ALLIANCE_THIRD;
                    }
                }
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV","OpenCV failed to load :( Error Code: " + errorCode);
                telemetry.update();
            }
        });

    }
}
