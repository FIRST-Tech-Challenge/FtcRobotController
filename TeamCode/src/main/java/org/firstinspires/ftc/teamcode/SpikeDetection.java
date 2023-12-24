package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="SpikeDetection")
public class SpikeDetection extends OpMode {

    Scalar red = new Scalar(255, 0, 0);
    Scalar blue = new Scalar(0, 0, 255);

    private SpikeDetectorPipeline spikeDetectorPipeline;
    private OpenCvCamera camera;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        spikeDetectorPipeline = new SpikeDetectorPipeline(blue);

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam);
        camera.setPipeline(spikeDetectorPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        dashboard.startCameraStream(camera, 0);
    }

    @Override
    public void loop() {
        dashboardTelemetry.addData("Spike Zone", spikeDetectorPipeline.getSpikeZone());
        dashboardTelemetry.update();
    }

}
