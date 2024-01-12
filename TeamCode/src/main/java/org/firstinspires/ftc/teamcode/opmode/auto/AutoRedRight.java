package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.robot.commands.tilt.TiltGoToPosition.TELEOP_DEPOSIT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.commands.claw.ClawCloseCommand;
import org.firstinspires.ftc.teamcode.robot.commands.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.robot.commands.tilt.TiltGoToPosition;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristDeposit;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristIntake;
import org.firstinspires.ftc.teamcode.robot.commands.wrist.WristStow;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.vision.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous(group="auto", name="Auto_RED_Backboard_side")
public class AutoRedRight extends LinearOpMode
{
    private static final double TILE = 24.0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ClawSubsystem clawSubsystem = new ClawSubsystem(hardwareMap);
        TiltSubsystem tiltSubsystem = new TiltSubsystem(hardwareMap, telemetry);
        WristSubsystem wristSubsystem = new WristSubsystem(hardwareMap);

        Pose2d startPose = new Pose2d(TILE/2, -2.5*TILE, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        ParallelCommandGroup initiate = new ParallelCommandGroup(
                new ClawCloseCommand(clawSubsystem),
                new WristStow(wristSubsystem)
        );
        SequentialCommandGroup place_pixel_and_stow = new SequentialCommandGroup(
                new TiltGoToPosition(tiltSubsystem, TiltGoToPosition.TELEOP_INTAKE),
                new WristIntake(wristSubsystem),
                new ClawOpenCommand(clawSubsystem, ClawOpenCommand.Side.LEFT),
                new WristStow(wristSubsystem));
        SequentialCommandGroup deposit = new SequentialCommandGroup(
                new TiltGoToPosition(tiltSubsystem, TELEOP_DEPOSIT),
                new WristDeposit(wristSubsystem),
                new ClawOpenCommand(clawSubsystem, ClawOpenCommand.Side.RIGHT));

        TrajectorySequence Center = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1.6*TILE))
                //.lineToConstantHeading(new Vector2d(TILE/2, -1.5*TILE))
                .turn(Math.toRadians(170))
                .addTemporalMarker(2, () -> {
                    telemetry.addData("RUNNING BEFORE", 0);
                    telemetry.update();
                    CommandScheduler.getInstance().schedule(place_pixel_and_stow);
                    telemetry.addData("RUNNING AFTER", 0);
                    telemetry.update();
                })
                .waitSeconds(4)
                .turn(Math.toRadians(-170+90))
                .lineToConstantHeading(new Vector2d(2*TILE, -1.5*TILE))
                /*.addTemporalMarker(5, () -> {
                    CommandScheduler.getInstance().schedule(deposit);
                })
                .waitSeconds(4)*/
                .lineToConstantHeading(new Vector2d(2*TILE, -3*TILE))
                .build();
                /////////////////


        TrajectorySequence Right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1*TILE))
                .turn(Math.toRadians(45))
                .addTemporalMarker(1, () -> {
                    CommandScheduler.getInstance().schedule(place_pixel_and_stow);
                })
                .turn(Math.toRadians(45))
                .lineToConstantHeading(new Vector2d(TILE/2, -2*TILE))

                .lineToConstantHeading(new Vector2d(2*TILE, -1.75*TILE))
                .addTemporalMarker(5, () -> {
                    CommandScheduler.getInstance().schedule(deposit);
                })
                .lineToConstantHeading(new Vector2d(2*TILE, -2.5*TILE))
                .build();

        TrajectorySequence Left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1*TILE))
                .turn(Math.toRadians(45))
                .addTemporalMarker(1, () -> {
                    CommandScheduler.getInstance().schedule(place_pixel_and_stow);
                })
                .turn(Math.toRadians(45))
                .lineToConstantHeading(new Vector2d(TILE/2, -2*TILE))
                .lineToConstantHeading(new Vector2d(2*TILE, -1.75*TILE))
                .addTemporalMarker(5, () -> {
                    CommandScheduler.getInstance().schedule(deposit);
                })
                .lineToConstantHeading(new Vector2d(2*TILE, -2.5*TILE))
                .build();

        TeamElementPipeline.MarkerPosistion markerPosistion;
        //Vision.startStreaming(hardwareMap, telemetry);
        markerPosistion = TeamElementPipeline.MarkerPosistion.CENTER;
        //markerPosistion = Vision.determineMarkerPosistion(4000);
        ////////////////////////////////////////////

        CommandScheduler.getInstance().schedule(initiate);
        waitForStart();


        if (isStopRequested()) return;

        /*telemetry.addData("DONE 2",0);
        telemetry.update();*/
        while(!isStopRequested()) {
            CommandScheduler.getInstance().run();
            switch (markerPosistion) {
                case CENTER:
                case UNKNOWN:
                    drive.followTrajectorySequence(Center);
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(Right);
                    break;
                case LEFT:
                    drive.followTrajectorySequence(Left);
                    break;
            }
        }
    }
}
