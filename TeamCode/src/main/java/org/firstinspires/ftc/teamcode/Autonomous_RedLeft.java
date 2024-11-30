package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClimberDownCommand;
import org.firstinspires.ftc.teamcode.commands.ClimberStopCommand;
import org.firstinspires.ftc.teamcode.commands.ClimberUpCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorRetractCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberDropCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberDropToggleCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberPickupCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberPickupToggleCommand;
import org.firstinspires.ftc.teamcode.commands.WormLowerCommand;
import org.firstinspires.ftc.teamcode.commands.WormRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.WristDownCommand;
import org.firstinspires.ftc.teamcode.commands.WristStopCommand;
import org.firstinspires.ftc.teamcode.commands.WristUpCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous: RedLeft", group="Scrimmage")
public class Autonomous_RedLeft extends OpMode {

    private Elevator elevator;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private SampleMecanumDrive drive;
    private Climber climber;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
        elevator = new Elevator(hardwareMap, telemetry);
        worm = new Worm(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);
        wrist.Goto(0);
        climber = new Climber(hardwareMap, telemetry);
        climber.Goto(0);

        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void start() {
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-36.18, -62.87, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-2.51, -33.67), Math.toRadians(0.0))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);

        //TODO: Implement the specimen drop code!

        trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-2.51, -33.67, Math.toRadians(0.0)))
                .splineTo(new Vector2d(55.34, -52.66), Math.toRadians(270.0))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);

        //TODO: Implement the speciment pick up code!

        trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(55.34, -52.66, Math.toRadians(270.0)))
                .splineTo(new Vector2d(58.57, -57.13), Math.toRadians(216.03))
                .splineTo(new Vector2d(-1.61, -32.06), Math.toRadians(180.0))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);

        //TODO: Implement the specimen drop code!

        trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-1.61, -32.06, Math.toRadians(180.0)))
                .splineTo(new Vector2d(-25.61, -29.37), Math.toRadians(111.80))
                .splineTo(new Vector2d(-25.25, 1.61), Math.toRadians(90.56))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);
    }

    @Override
    public void loop() {

    }
}
