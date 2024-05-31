package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.BackdropAlign;
import org.firstinspires.ftc.teamcode.commands.ClimbDefault;
import org.firstinspires.ftc.teamcode.commands.DriveDefault;
import org.firstinspires.ftc.teamcode.commands.IntakeDefault;
import org.firstinspires.ftc.teamcode.commands.RunIntake;
import org.firstinspires.ftc.teamcode.commands.SlideCalibrate;
import org.firstinspires.ftc.teamcode.commands.SlideToPosition;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.InstantCommand;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Trigger;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.WaitCommand;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;

@TeleOp(name = "TeleOp")
public class Tele extends Robot {
    public static Pose2d backdropPose = blueBackdropPose;
    Command automaticPlace;

    @Override
    public void setup() {
        aprilTagCamera.enable();
        if (alliance == Alliance.RED) {
            backdropPose = redBackdropPose;
        }
        automaticPlace = getAutomaticPlaceCommand(backdropPose.toWaypoint());

        drive.setDefaultCommand(new DriveDefault(drive, () -> -driveController.leftStickY.getAsDouble(), () -> driveController.leftStickX.getAsDouble(), () -> -driveController.rightStickX.getAsDouble()));
        driveController.leftBumper.onTrue(new InstantCommand(() -> drive.enableSlowMode()));
        driveController.rightBumper.onTrue(new InstantCommand(() -> drive.enableFastMode()));
        driveController.dpadUp.onTrue(new InstantCommand(() -> climber.deliverHook()));
        driveController.dpadDown.onTrue(new InstantCommand(() -> climber.hookDown()));
        BackdropAlign backdropAlign = new BackdropAlign(drive, placer, 2000);
        driveController.a.onTrue(backdropAlign);
        driveController.gamepadActive.and(new Trigger(() -> backdropAlign.timeSinceInitialized() > 1000)).onTrue(new InstantCommand((() -> backdropAlign.cancel())));
        driveController.b.and(driveController.x).and(driveController.y).onTrue(new InstantCommand(() -> drive.odometry.setPosition(new Pose2d())));

        if (botPose == null) {
            botPose = new Pose2d();
        }
        drive.odometry.setPosition(botPose); // Set the robot position to the last position of the robot in autonomous
        drive.setFieldCentricOffset(fieldCentricOffset);

        intake.setDefaultCommand(new IntakeDefault(intake, lights, drive.odometry::getPose)); // Runs the intake automatically when the robot is in the right spot
        payloadController.rightTrigger.or(driveController.rightTrigger).whileTrue(new RunIntake(intake, SubsystemConstants.Intake.defaultSpeed));
        payloadController.leftTrigger.or(driveController.leftTrigger).whileTrue(new RunIntake(intake, -SubsystemConstants.Intake.defaultSpeed));

        Command shootDrone = new SequentialCommandGroup(
                new InstantCommand(() -> droneShooter.shootAngle()),
                new WaitCommand(2000),
                new InstantCommand(() -> {
                    droneShooter.releaseAngle();
                })
        );

        payloadController.rightBumper.onTrue(new SlideToPosition(slide, SubsystemConstants.Slide.defaultPlacePosition));
        payloadController.options.whileTrue(new SlideCalibrate(slide));
        payloadController.options.and(payloadController.b).onTrue(new InstantCommand(() -> slide.encoder.reset()));
        slide.encoder.setPosition(slidePose); // Set the slide position to the last slide position in autonomous

        payloadController.a.onTrue(new InstantCommand(() -> placer.open()));
        payloadController.b.onTrue(new InstantCommand(() -> placer.close()));

        climber.setDefaultCommand(new ClimbDefault(climber, payloadController.leftStickY));
        payloadController.dpadRight.onTrue(new InstantCommand(() -> climber.deliverHook()));
        payloadController.dpadLeft.onTrue(new SlideToPosition(slide, SubsystemConstants.Slide.defaultPlacePosition));

        payloadController.dpadDown.onTrue(new InstantCommand(() -> placer.storagePosition()));
        payloadController.dpadUp.onTrue(new InstantCommand(() -> placer.placePosition()));

        payloadController.y.onTrue(shootDrone);
        payloadController.rightBumper.onTrue(new SlideToPosition(slide, SubsystemConstants.Slide.defaultPlacePosition));
        payloadController.leftBumper.onTrue(new SlideToPosition(slide, -100));
    }

    @Override
    public void onStart() {
        droneShooter.storeAngle();
    }

    @Override
    public void mainLoop() {
        telemetry.addData("servo pose", droneShooter.angleAdjuster.getPosition());
        telemetry.addData("heading", drive.odometry.getPose().rotation.getAngleDegrees());
    }
}
