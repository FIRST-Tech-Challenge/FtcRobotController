package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous: Park in Ascent Zone", group="000Real")
public class Autonomous_Park_AscentZone extends OpMode {

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
        wrist = new Wrist(hardwareMap, telemetry, true);
        wrist.Goto(0);
        climber = new Climber(hardwareMap, telemetry);
        climber.Goto(0);

        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void start() {
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-35.02, -62.93, Math.toRadians(90)))
                .splineTo(new Vector2d(-35.56, -16.18), Math.toRadians(90))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);
    }

    @Override
    public void loop() {

    }
}
