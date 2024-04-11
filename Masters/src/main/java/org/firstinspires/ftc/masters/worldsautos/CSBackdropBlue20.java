package org.firstinspires.ftc.masters.worldsautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CenterStagePilesBlue;
import org.firstinspires.ftc.masters.PropFindRight;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp(name = "CSBackdropBlue20", group = "CS")
public class CSBackdropBlue20 extends LinearOpMode {

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,
        RETRACT_SLIDE,
        DRIVE_TO_STACK,
        PICK_UP_FROM_STACK,
        DRIVE_TO_BACKBOARD,
        DROP_WHITE,
        YELLOW_DEPOSIT,
        DROP_YELLOW,
        TO_STACK_CYCLE,

        BACK,
        SPIN,
        PARK,
        STOP
    }

    ElapsedTime depositTime = new ElapsedTime();
    int resetInt = 0;

    SampleMecanumDrive drive;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontWebcam"), cameraMonitorViewId);
        PropFindRight myPipeline;
        webcam.setPipeline(myPipeline = new PropFindRight(telemetry, packet));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PropFindRight.pos propPos = null;

        CSBackdropBlue20.State currentState;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        TrajectorySequence wallToLeftSpike = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(180)), Math.toRadians(-70))
                .build();

        TrajectorySequence LeftSpikeToBackdrop = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, 29, Math.toRadians(180)), Math.toRadians(0))
                .build();

        waitForStart();

    }
}
