package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@Config


@Autonomous
public class BlueScoring extends LinearOpMode {

    protected MecanumDrive drive;
    // TODO: create instances of subsystems
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;
    PoseMap poseMap;
    private double feederOnTime = 2;


    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(14, 60.5, Math.toRadians(270));

        // region block 1
        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)
                                .stopAndAdd(() -> shooterSubsystem.runShooter() ) // turn on shooter
                                .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // shoot
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                                .stopAndAdd(() -> intakeSubsystem.runIntakeForward()) //turn on intake
                                .strafeToSplineHeading(new Vector2d(60.5,60.5), Math.toRadians(270)) // intake
                                .waitSeconds(.5)
                                .stopAndAdd(() -> intakeSubsystem.stopIntake()) // turn off intake
                                .strafeToLinearHeading(new Vector2d(55,55),Math.toRadians(220)) // camera check
                                .waitSeconds(1)
                                .build()
                )

        );
        // endregion

        if (cameraTrackingSubsystems.seeApirltag()){
            // region block 2
            Actions.runBlocking(
                    new RaceAction(
                            drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(220)), poseMap)

                                    .setTangent(180)
                                    .splineToLinearHeading(new Pose2d(14, 60.5, Math.toRadians(270)), Math.toRadians(180)) // move to garden side
                                    .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // pew pew
                                    .waitSeconds(feederOnTime)
                                    .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooter
                                    .waitSeconds(1) // camera check

                                    .build()
                    )

            );
            // endregion

            if (cameraTrackingSubsystems.seeApirltag()){
                // region block 4
                Actions.runBlocking(
                        new RaceAction(
                                drive.actionBuilder(new Pose2d(14, 60.5, Math.toRadians(270)), poseMap)

                                        .setTangent(Math.toRadians(330))
                                        .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(300))

                                        .build()
                        )


                );

                // endregion
            } else {
                // region block 5
                Actions.runBlocking(
                        new RaceAction(
                                drive.actionBuilder(new Pose2d(14, 60.5, Math.toRadians(270)), poseMap)

                                        .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(130)), Math.toRadians(250)) //move to loading side
                                        .stopAndAdd(() -> intakeSubsystem.runIntakeForward())
                                        .waitSeconds(3) // intake human player
                                        .stopAndAdd(() -> intakeSubsystem.stopIntake()) // stop intaking
                                        .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // start shooting
                                        .waitSeconds(2) // shoot
                                        .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                                        .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(300)) // park

                                        .build()
                        )


                );

                // endregion//
            }

        } else {
            // region block 3
            Actions.runBlocking(
                    new RaceAction(
                            drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(220)), poseMap)

                                    .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(130)), Math.toRadians(250)) //move to loading side
                                    .stopAndAdd(() -> feederSubsystem.runIndexerForward())  //shoot
                                    .waitSeconds(feederOnTime)
                                    .stopAndAdd(() -> feederSubsystem.stopIndexer()) //stop shooting
                                    .stopAndAdd(() -> intakeSubsystem.runIntakeForward()) //start intaking
                                    .waitSeconds(3)
                                    .stopAndAdd(() -> intakeSubsystem.stopIntake()) //stop intaking
                                    .waitSeconds(1) // camera check

                                    .build()
                    )

            );
            // endregion

            if (cameraTrackingSubsystems.seeApirltag()){
                // region block 6
                Actions.runBlocking(
                        new RaceAction(
                                drive.actionBuilder(new Pose2d(40, -45, Math.toRadians(130)), poseMap)
                                        .setTangent(Math.toRadians(80))
                                        .splineToLinearHeading(new Pose2d(14, 60.5, Math.toRadians(270)), Math.toRadians(160)) //move to garden side
                                        .stopAndAdd(() -> feederSubsystem.runIndexerForward()) // shoot
                                        .waitSeconds(feederOnTime)
                                        .stopAndAdd(() -> feederSubsystem.stopIndexer()) // stop shooting
                                        .setTangent(Math.toRadians(320))
                                        .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(300)) //park


                                        .build()
                        )

                );
                // endregion
            } else {
                // region block 7
                Actions.runBlocking(
                        new RaceAction(
                                drive.actionBuilder(new Pose2d(40, -45, Math.toRadians(130)), poseMap)
                                        .stopAndAdd(() -> feederSubsystem.runIndexerForward())
                                        .waitSeconds(feederOnTime) // shoot
                                        .stopAndAdd(() -> feederSubsystem.stopIndexer())
                                        .setTangent(Math.toRadians(50))
                                        .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(50)) //park

                                        .build()
                        )


                );
                // endregion
            }

        }
    }
}
