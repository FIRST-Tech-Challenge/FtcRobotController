package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Blue Close")
public class BlueClose extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;
    Slides s;
    Outtake o;
    V4B v4b;
    GamepadEx gamepad;

    public static Pose2d startPoseCloseBlue = new Pose2d(12,62, Math.toRadians(-90));

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        s = new Slides(hardwareMap);
        o = new Outtake(gamepad, hardwareMap);
        v4b = new V4B(hardwareMap);


        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Waiting For Start...");
        }

        TrajectorySequence propCloseMidBlue = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(25)
                .lineToLinearHeading(new Pose2d(12, 62, 0))
                .build();

        TrajectorySequence scoreCloseMidBlue = drive.trajectorySequenceBuilder(propCloseMidBlue.end())
                .forward(34)
                .strafeRight(19)
                .build();

        TrajectorySequence parkCloseMidBlue = drive.trajectorySequenceBuilder(scoreCloseMidBlue.end())
                .strafeLeft(25)
                .forward(12)
                .build();

//        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like norma
//            new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                    new TrajectorySequenceCommand(drive, propCloseMidBlue),
//                    new TrajectorySequenceCommand(drive, scoreCloseMidBlue),
//                    new TrajectorySequenceCommand(drive, parkCloseMidBlue)
//                ),
//                new InstantCommand(() -> {
//                    new WaitCommand(4000);
//                    s.goToPosition(Slides.SlidePos.LOW);
////                    new WaitCommand(2000);
////                    v4b.togglePower();
////                    new WaitCommand(2000);
////                    o.open();
////                    new WaitCommand(2000);
////                    o.close();
////                    v4b.togglePower();
////                    new WaitCommand(1000);
////                    s.goToPosition(Slides.SlidePos.DOWN);
//                })
//            )
//        ));

        schedule(new SequentialCommandGroup(
//                new TrajectorySequenceCommand(drive, propCloseMidBlue),
                new InstantCommand(() -> {
                    s.runToPos(Slides.SlidePos.LOW.position);
                })
//                new TrajectorySequenceCommand(drive, scoreCloseMidBlue),
//                new TrajectorySequenceCommand(drive, parkCloseMidBlue)
        ));
    }
}