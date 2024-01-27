package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subbys.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.vision.pipeline.CapstoneDetector;

@Autonomous(name = "VisionPixel1")
public class VisionPixel1 extends CommandOpMode {

    private CapstoneDetector capstoneDetector;

    private MecanumDriveSubsystem drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(35.0, -62.0, Math.toRadians(0.0));

    @Override
    public void initialize() {

        capstoneDetector = new CapstoneDetector(hardwareMap, "coolio");
        capstoneDetector.init();

        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        backRight.motor.setDirection(DcMotor.Direction.FORWARD);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        if(capstoneDetector.getPlacement() == CapstoneDetector.Placement.LEFT){
            Trajectory center = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                    .back(28)
                    .build();
            Trajectory left = drive.trajectoryBuilder(new Pose2d(12.55, -64.31), Math.toRadians(90))
                    .strafeRight(17)
                    .build();
            Trajectory back = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                    .back(5)
                    .build();

            TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, center);
            TrajectoryFollowerCommand autonomous2 = new TrajectoryFollowerCommand(drive, left);
            TrajectoryFollowerCommand autonomous3 = new TrajectoryFollowerCommand(drive, back);

            if(isStopRequested()){

            }

            schedule(autonomous.andThen(autonomous2).andThen(autonomous3));

        }else if(capstoneDetector.getPlacement() == CapstoneDetector.Placement.CENTER){
            Trajectory center = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                    .back(35)
                    .build();

            Trajectory back = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                    .forward(5)
                    .build();
            TrajectoryFollowerCommand autonomous2 = new TrajectoryFollowerCommand(drive, back);

            TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, center);

            if(isStopRequested()){

            }

            schedule(autonomous.andThen(autonomous2));
        }else{
            Trajectory center = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                    .back(28)
                    .build();
            Trajectory left = drive.trajectoryBuilder(new Pose2d(12.55, -64.31), Math.toRadians(90))
                    .strafeLeft(23.7)
                    .build();
            Trajectory back = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                    .forward(5)
                    .build();

            TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, center);
            TrajectoryFollowerCommand autonomous2 = new TrajectoryFollowerCommand(drive, left);
            TrajectoryFollowerCommand autonomous3 = new TrajectoryFollowerCommand(drive, back);

            if(isStopRequested()){

            }

            schedule(autonomous.andThen(autonomous2).andThen(autonomous3));
        }

    }
}