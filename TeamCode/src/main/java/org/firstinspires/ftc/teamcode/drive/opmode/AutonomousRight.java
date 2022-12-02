package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SignalDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "autonomous", name = "Autonomous")
public class AutonomousRight extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private SignalDetection pipeline = null;
    private String webcamName = "Webcam";
    private boolean useCameraStream = true;
    private OpenCvWebcam camera = null;
    private Telemetry.Item camTelemetry = null;
    private Telemetry.Item colorTelemetry = null;
    private Telemetry.Item timeTelemetry = null;
    private SignalDetection.Color color = null;
    private long msUntilDetected = 0;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory standardTraj = drive.trajectoryBuilder(new Pose2d())
                .forward(100)
                .build();

        Trajectory leftTraj = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(100)
                .build();

        Trajectory rightTraj = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(100)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pipeline = new SignalDetection(webcamName, camTelemetry, runtime);
//        pipeline = new DuckPipeline();

        //Get webcam and init OpenCV on webcam
        WebcamName webcam = hardwareMap.get(WebcamName.class, webcamName);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if (useCameraStream) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam);
        }

        // Specify the pipeline the camera is using
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                // nope
            }
        });

        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
        colorTelemetry = telemetry.addData("Color", null);
        timeTelemetry = telemetry.addData("Time until detected", null);
        while (!isStarted()) {
//            telemetry.addData("ur in", "init loop");
            color = pipeline.getColor();
            msUntilDetected = pipeline.getMsUntilDetected();
            colorTelemetry.setValue("Color: " + color);
            timeTelemetry.setValue(msUntilDetected);
            telemetry.update();
        }

        if(isStopRequested()) return;

        drive.followTrajectory(standardTraj);
//        switch (color) {
//            case "" {
//                // path
//            }
//        }
    }
}