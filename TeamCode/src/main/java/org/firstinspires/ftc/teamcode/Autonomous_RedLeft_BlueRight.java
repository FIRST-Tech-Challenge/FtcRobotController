package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous: Red Left/Blue Right", group="000Real")
public class Autonomous_RedLeft_BlueRight extends CommandOpMode {

    private Elevator elevator;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private SampleMecanumDrive drive;
    private Climber climber;

    public void initalize() {
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

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-35.38, -63.64, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-43.73, -43.02), Math.toRadians(224.15))
                .build();

        //wrist.setAngle(-10);
        //worm.goToAngle(96);
        // elevator 33in

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-43.73, -43.02, Math.toRadians(224.15)))
                .splineTo(new Vector2d(-29.87, -12.09), Math.toRadians(25.02))
                .build();

        schedule(new SequentialCommandGroup(
                new TrajectorySequenceFollower
        ));
    }
}
