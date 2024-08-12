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
import org.rustlib.commandsystem.Command;
import org.rustlib.commandsystem.InstantCommand;
import org.rustlib.commandsystem.SequentialCommandGroup;
import org.rustlib.commandsystem.Trigger;
import org.rustlib.commandsystem.WaitCommand;
import org.rustlib.geometry.Pose2d;

@TeleOp(name = "TeleOp")
public class Tele extends Robot {
    public static Pose2d backdropPose = blueBackdropPose;
    Command automaticPlace;

    @Override
    public void onInit() {
        aprilTagCamera.enable();
        if (alliance == Alliance.RED) {
            backdropPose = redBackdropPose;
        }
        automaticPlace = getAutomaticPlaceCommand(backdropPose.toWaypoint());

        drive.setDefaultCommand(new DriveDefault(drive, () -> -controller1.leftStickY.getAsDouble(), () -> controller1.leftStickX.getAsDouble(), () -> -controller1.rightStickX.getAsDouble()));
        controller1.leftBumper.onTrue(new InstantCommand(() -> drive.enableSlowMode()));
        controller1.rightBumper.onTrue(new InstantCommand(() -> drive.enableFastMode()));
        controller1.dpadUp.onTrue(new InstantCommand(() -> climber.deliverHook()));
        controller1.dpadDown.onTrue(new InstantCommand(() -> climber.hookDown()));
        BackdropAlign backdropAlign = new BackdropAlign(drive, placer, 2000);
        controller1.a.onTrue(backdropAlign);
        controller1.gamepadActive.and(new Trigger(() -> backdropAlign.timeSinceInitialized() > 1000)).onTrue(new InstantCommand((backdropAlign::cancel)));
        controller1.b.and(controller1.x).and(controller1.y).onTrue(new InstantCommand(() -> drive.getOdometry().setPosition(new Pose2d())));

        if (botPose == null) {
            botPose = new Pose2d();
        }
        drive.getOdometry().setPosition(botPose); // Set the robot position to the last position of the robot in autonomous
        drive.setFieldCentricOffset(fieldCentricOffset); // TODO: make way to set field centric offset

        intake.setDefaultCommand(new IntakeDefault(intake, lights, drive.getOdometry()::getPose)); // Runs the intake automatically when the robot is in the right spot
        controller2.rightTrigger.or(controller1.rightTrigger).whileTrue(new RunIntake(intake, SubsystemConstants.Intake.defaultSpeed));
        controller2.leftTrigger.or(controller1.leftTrigger).whileTrue(new RunIntake(intake, -SubsystemConstants.Intake.defaultSpeed));

        Command shootDrone = new SequentialCommandGroup(
                new InstantCommand(() -> droneShooter.shootAngle()),
                new WaitCommand(2000),
                new InstantCommand(() -> {
                    droneShooter.releaseAngle();
                })
        );

        controller2.rightBumper.onTrue(new SlideToPosition(slide, SubsystemConstants.Slide.defaultPlacePosition));
        controller2.options.whileTrue(new SlideCalibrate(slide));
        controller2.options.and(controller2.b).onTrue(new InstantCommand(() -> slide.encoder.reset()));
        slide.encoder.setTicks(slidePose); // Set the slide position to the last slide position in autonomous

        controller2.a.onTrue(new InstantCommand(() -> placer.open()));
        controller2.b.onTrue(new InstantCommand(() -> placer.close()));

        climber.setDefaultCommand(new ClimbDefault(climber, controller2.leftStickY));
        controller2.dpadRight.onTrue(new InstantCommand(() -> climber.deliverHook()));
        controller2.dpadLeft.onTrue(new SlideToPosition(slide, SubsystemConstants.Slide.defaultPlacePosition));

        controller2.dpadDown.onTrue(new InstantCommand(() -> placer.storagePosition()));
        controller2.dpadUp.onTrue(new InstantCommand(() -> placer.placePosition()));

        controller2.y.onTrue(shootDrone);
        controller2.rightBumper.onTrue(new SlideToPosition(slide, SubsystemConstants.Slide.defaultPlacePosition));
        controller2.leftBumper.onTrue(new SlideToPosition(slide, -100));
    }

    @Override
    public void onStart() {
        droneShooter.storeAngle();
    }

    @Override
    public void mainLoop() {
        telemetry.addData("servo pose", droneShooter.angleAdjuster.getPosition());
        telemetry.addData("heading", drive.getOdometry().getPose().rotation.getAngleDegrees());
    }
}
