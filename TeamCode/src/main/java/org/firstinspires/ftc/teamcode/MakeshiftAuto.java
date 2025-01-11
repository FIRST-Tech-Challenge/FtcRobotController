package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non- RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class MakeshiftAuto extends LinearOpMode {
    private BaseRobot baseRobot;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.5, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;


        TrajectoryActionBuilder MoveSampleToHumanPlayerZone = drive.actionBuilder(new Pose2d(11.5, -60, Math.toRadians(270))).endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36, -35), Math.toRadians(90))
                .strafeTo(new Vector2d(36,-13))
                .strafeTo(new Vector2d(48,-13))
                .strafeTo(new Vector2d(48,-50))
                .strafeTo(new Vector2d(48,-13))
                .strafeTo(new Vector2d(58,-13))
                .strafeTo(new Vector2d(58,-50));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action PlaceSample = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(5, -30), Math.toRadians(270))
                .build();
        Action trajectoryActionCloseOut = MoveSampleToHumanPlayerZone.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = MoveSampleToHumanPlayerZone.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        PlaceSample,
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
    }
}
