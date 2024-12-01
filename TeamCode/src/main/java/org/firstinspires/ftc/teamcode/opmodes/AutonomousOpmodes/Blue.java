package org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class Blue extends LinearOpMode {
    int visionOutputPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
            if (isStopRequested()) return;
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        TrajectoryActionBuilder red = drive.actionBuilder(new Pose2d(7.00, -70.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(7.00, -25.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(7.00, -45.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(50.00, -35.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(50.00, -60.00), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(60.00, -60.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(60.00, -35.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(48.00, -52.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(48.00, -70.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(7.00, -25.00), Math.toRadians(90.00));



        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        }
    }
}
