package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Config
@Autonomous
public class BioBuzzAutoBackTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Define your robot's starting position (X, Y in inches, and Heading in radians)
        Pose2d beginPose = new Pose2d(0, 0, 0);

        // Initialize the RoadRunner MecanumDrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        telemetry.addData("Status", "Initialized. Waiting for start.");
        telemetry.update();



        waitForStart();

        if (isStopRequested()) return;

        // Drive a 24x24 inch square using absolute field coordinates
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-6, 0))

                        .build()

        );
    }
}


