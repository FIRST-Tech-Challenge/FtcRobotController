package org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.util.AutonomousActions;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class Blue extends LinearOpMode {
    int visionOutputPosition = 0;

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

        // Trajectories

        AutonomousActions.Pincher pincher = new AutonomousActions.Pincher(hardwareMap);
        AutonomousActions.Uppies uppies = new AutonomousActions.Uppies(hardwareMap);

        TrajectoryActionBuilder red = drive.actionBuilder(new Pose2d(7.00, -70.00, Math.toRadians(90.00)))
                .lineToY(-35) // (7, -25) 90 deg
                // Put Specimen on rung
                .afterTime(.5f, new SequentialAction(
                        uppies.iWantUp(),
                        uppies.iWantDown(),
                        pincher.openPincher()
                    )
                )
                .strafeTo(new Vector2d(50.00, -35.00)) // (50, -35) 90 deg
                // Grab 1st sample
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(50, -60), Math.toRadians(-90)) // (50, -60) -90 deg
                // Drop off 1st sample
                .waitSeconds(3)
                .turnTo(Math.toRadians(90)) // 90 deg
                .strafeTo(new Vector2d(60, -60.00)) // (60, -60) 90 deg
                .strafeTo(new Vector2d(60, -35.00)) // (60, -35) 90 deg
                // Grab 2nd Sample
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(60, -60), Math.toRadians(-90)) // (60, -60) -90 deg
                // Drop off Second Sample
                .waitSeconds(3)
                .strafeTo(new Vector2d(48.00, -52.00)) // (48, -52)
                // Wait for human player to put specimen on wall
                .waitSeconds(3)
                .strafeTo(new Vector2d(48, -60)) // (48, -80) -90 deg
                // Move and pickup specimen
                .afterTime(.5f, pincher.closePincher())
                .waitSeconds(3)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(7.00, -35))
                // Put specimen on rung
                .afterTime(.5f, new SequentialAction(
                                uppies.iWantUp(),
                                uppies.iWantDown(),
                                pincher.openPincher()
                        )
                )
                .waitSeconds(3);
        Actions.runBlocking(red.build());
    }
}
