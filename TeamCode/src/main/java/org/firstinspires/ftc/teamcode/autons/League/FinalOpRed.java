package org.firstinspires.ftc.teamcode.autons.League;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subbys.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.vision.pipeline.CapstoneDetector;
import org.firstinspires.ftc.teamcode.vision.pipeline.CapstoneDetectorBlue;

import java.util.HashMap;

@Autonomous(name = "FINALOPBLUELEAGUE")
public class FinalOpRed extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private SimpleServo left_claw, right_claw;
    private Motor hang, arm;
//    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(12, -61, Math.toRadians(90));
    private CapstoneDetectorBlue capstoneDetector;
//    private CapstoneDetector.Placement place;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(startPose);

        left_claw = new SimpleServo(hardwareMap, "rc", -180, 180);
        right_claw = new SimpleServo(hardwareMap, "lc", -180, 180);

        hang = new Motor(hardwareMap, "hang");
//        sensor = new SensorRevTOFDistance(hardwareMap, "commonSense");
        arm = new Motor(hardwareMap, "arm");

        Trajectory path1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(13.5, -33), Math.toRadians(45))
                .build();
        Trajectory path2 = drive.trajectoryBuilder(path1.end())
                .back(17.5)
                .build();
        Trajectory path3 = drive.trajectoryBuilder(path2.end())
                .splineToLinearHeading(new Pose2d(-20.25, -30, Math.toRadians(180)), Math.toRadians(180))
                .build();
        Trajectory path4 = drive.trajectoryBuilder(path3.end())
                .forward(14)
                .build();
        Trajectory path5 = drive.trajectoryBuilder(path4.end())
                .back(0)
                .build();
        Trajectory path6 = drive.trajectoryBuilder(path5.end())
                .back(8)
                .build();
        Trajectory path7 = drive.trajectoryBuilder(path6.end())
                .strafeLeft(38)
                .build();
        Trajectory path8 = drive.trajectoryBuilder(path7.end())
                .forward(15)
                .build();

        Trajectory path9 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-3.5, -36))
                .build();
        Trajectory path10 = drive.trajectoryBuilder(path9.end())
                .back(17.5)
                .build();
        Trajectory path11 = drive.trajectoryBuilder(path10.end())
                .splineToLinearHeading(new Pose2d(-22, -33.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
        Trajectory path12 = drive.trajectoryBuilder(path11.end())
                .forward(14)
                .build();
        Trajectory path13 = drive.trajectoryBuilder(path12.end())
                .back(2)
                .build();
        Trajectory path14 = drive.trajectoryBuilder(path13.end())
                .back(8)
                .build();
        Trajectory path15 = drive.trajectoryBuilder(path14.end())
                .strafeLeft(30)
                .build();
        Trajectory path16 = drive.trajectoryBuilder(path15.end())
                .forward(15)
                .build();

        Trajectory path17 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, -31))
                .build();
        Trajectory path18 = drive.trajectoryBuilder(path17.end())
                .back(17.5)
                .build();
        Trajectory path19 = drive.trajectoryBuilder(path18.end())
                .splineToLinearHeading(new Pose2d(-21, -28.8, Math.toRadians(180)), Math.toRadians(180))
                .build();
        Trajectory path20 = drive.trajectoryBuilder(path19.end())
                .forward(14)
                .build();
        Trajectory path21 = drive.trajectoryBuilder(path20.end())
                .back(2)
                .build();
        Trajectory path22 = drive.trajectoryBuilder(path21.end())
                .back(8)
                .build();
        Trajectory path23 = drive.trajectoryBuilder(path22.end())
                .strafeLeft(34)
                .build();
        Trajectory path24 = drive.trajectoryBuilder(path23.end())
                .forward(15)
                .build();
        capstoneDetector = new CapstoneDetectorBlue(hardwareMap, "coolio");
        capstoneDetector.init();

        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new WaitCommand(5000))
                .andThen(new SelectCommand(new HashMap<Object, Command>(){{
                    put(CapstoneDetectorBlue.Placement.RIGHT,
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> hang.set(-1)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(0)),
                                    new WaitCommand(500),
                                    new InstantCommand(() -> left_claw.turnToAngle(-165)),
                                    new InstantCommand(() -> right_claw.turnToAngle(-10)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(1)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(0)),
                                    new TrajectoryFollowerCommand(drive, path1),
                                    new InstantCommand(() -> left_claw.turnToAngle(-10)),
                                    new TrajectoryFollowerCommand(drive, path2),
                                    new InstantCommand(() -> arm.set(-0.65)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> arm.set(-0.5)),
                                    new WaitCommand(400),
                                    new InstantCommand(() -> arm.set(0)),
                                    new TrajectoryFollowerCommand(drive, path3),
                                    new TrajectoryFollowerCommand(drive, path4),
                                    new InstantCommand(() -> arm.set(0.5)),
                                    new WaitCommand(900),
                                    new InstantCommand(() -> arm.set(0)),
                                    new TrajectoryFollowerCommand(drive, path5),
                                    new WaitCommand(0),
                                    new InstantCommand(() -> right_claw.turnToAngle(-160)),
                                    new WaitCommand(500),
                                    new TrajectoryFollowerCommand(drive, path6),
                                    new TrajectoryFollowerCommand(drive, path7),
                                    new TrajectoryFollowerCommand(drive, path8)
                            )
                    );
                    put(CapstoneDetectorBlue.Placement.CENTER,
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> hang.set(-1)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(0)),
                                    new WaitCommand(500),
                                    new InstantCommand(() -> left_claw.turnToAngle(-165)),
                                    new InstantCommand(() -> right_claw.turnToAngle(-10)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(1)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(0)),
                                    new TrajectoryFollowerCommand(drive, path17),
                                    new InstantCommand(() -> left_claw.turnToAngle(-10)),
                                    new TrajectoryFollowerCommand(drive, path18),
                                    new InstantCommand(() -> arm.set(-0.65)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> arm.set(-0.5)),
                                    new WaitCommand(400),
                                    new InstantCommand(() -> arm.set(0)),
                                    new TrajectoryFollowerCommand(drive, path19),
                                    new TrajectoryFollowerCommand(drive, path20),
                                    new InstantCommand(() -> arm.set(0.5)),
                                    new WaitCommand(1600),
                                    new InstantCommand(() -> arm.set(0)),
                                    new WaitCommand(1000),
                                    new TrajectoryFollowerCommand(drive, path21),
                                    new InstantCommand(() -> right_claw.turnToAngle(-160)),
                                    new WaitCommand(500),
                                    new TrajectoryFollowerCommand(drive, path22),
                                    new TrajectoryFollowerCommand(drive, path23),
                                    new TrajectoryFollowerCommand(drive, path24)
                            )
                    );
                    put(CapstoneDetectorBlue.Placement.LEFT,
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> hang.set(-1)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(0)),
                                    new WaitCommand(500),
                                    new InstantCommand(() -> left_claw.turnToAngle(-165)),
                                    new InstantCommand(() -> right_claw.turnToAngle(-10)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(1)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> hang.set(0)),
                                    new TrajectoryFollowerCommand(drive, path9),
                                    new InstantCommand(() -> left_claw.turnToAngle(-10)),
                                    new TrajectoryFollowerCommand(drive, path10),
                                    new InstantCommand(() -> arm.set(-0.65)),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> arm.set(-0.5)),
                                    new WaitCommand(400),
                                    new InstantCommand(() -> arm.set(0)),
                                    new TrajectoryFollowerCommand(drive, path11),
                                    new TrajectoryFollowerCommand(drive, path12),
                                    new InstantCommand(() -> arm.set(0.5)),
                                    new WaitCommand(1600),
                                    new InstantCommand(() -> arm.set(0)),
                                    new TrajectoryFollowerCommand(drive, path13),
                                    new WaitCommand(1000),
                                    new InstantCommand(() -> right_claw.turnToAngle(-160)),
                                    new InstantCommand(() -> arm.set(-0.5)),
                                    new WaitCommand(500),
                                    new TrajectoryFollowerCommand(drive, path14),
                                    new TrajectoryFollowerCommand(drive, path15),
                                    new TrajectoryFollowerCommand(drive, path16)
                            )
                    );
                }}, () -> capstoneDetector.getPlacement()))
        );


        schedule(new WaitCommand(500).andThen(new RunCommand(() -> {
            telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
            telemetry.update();
        })));
    }
}