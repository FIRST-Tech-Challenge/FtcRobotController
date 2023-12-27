package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "Center Stage Backdrop Blue", group = "competition")
public class CenterStageBackdropBlue extends LinearOpMode {
    enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,

        BACKUP_FROM_SPIKES,

        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT,

        PARK
    }

    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CenterStageComputerVisionPipelines CV = new CenterStageComputerVisionPipelines(hardwareMap, telemetry);
        CenterStageComputerVisionPipelines.pos propPos = null;

        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(12, 58.5), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        State currentState;

        Trajectory purpleDepositPathL = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(new Vector2d(12, 32.5), Math.toRadians(330)))
                .build();

        Trajectory purpleDepositPathR = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(new Vector2d(12, 32.5), Math.toRadians(210)))
                .build();

        Trajectory purpleDepositPathC = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(new Vector2d(12, 32.5), Math.toRadians(270)))
                .build();

        Trajectory backUpFromSpikes = drive.trajectoryBuilder(purpleDepositPathC.end())
                .back(20)
                .build();

        Trajectory yellowDepositPath = drive.trajectoryBuilder(backUpFromSpikes.end())
                .splineToLinearHeading(new Pose2d(new Vector2d(46,36),Math.toRadians(180)),Math.toRadians(-60))
                .build();

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(new Vector2d(56,56),Math.toRadians(180)),Math.toRadians(0))
                .build();

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = CV.propFind.position;
            telemetry.addData("Position", propPos);
        }

        currentState = State.PURPLE_DEPOSIT_PATH;
        if (propPos == CenterStageComputerVisionPipelines.pos.LEFT) {
            drive.followTrajectoryAsync(purpleDepositPathL);
        } else if (propPos == CenterStageComputerVisionPipelines.pos.RIGHT) {
            drive.followTrajectoryAsync(purpleDepositPathR);
        } else {
            drive.followTrajectoryAsync(purpleDepositPathC);
        }
    }
}
