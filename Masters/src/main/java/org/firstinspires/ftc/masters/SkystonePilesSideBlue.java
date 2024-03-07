package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.List;

//@Autonomous(name = "CenterStage Piles Blue", group = "competition")
public class SkystonePilesSideBlue extends LinearOpMode {

    enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,

        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT,

        TOPARK,

        PARK,
        LOWER,
        END
    }

    SampleMecanumDrive drive;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(-35, 58.5), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        CenterStagePilesBlue.State currentState;

        Trajectory blank = drive.trajectoryBuilder(startPose,false)
                .lineToSplineHeading(new Pose2d(-35, 10, Math.toRadians(90)))
                .build();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        }

    }

}