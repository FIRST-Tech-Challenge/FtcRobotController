package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subbys.MecanumDriveSubsystem;

@Autonomous(name = "Right")
public class RightPathTest extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

//    private Pose2d startPose = new Pose2d(35.0, -62.0, Math.toRadians(0.0));

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        frontLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        frontRight.motor.setDirection(DcMotor.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotor.Direction.FORWARD);
        backRight.motor.setDirection(DcMotor.Direction.FORWARD);

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        Trajectory right = drive.trajectoryBuilder(new Pose2d(12.55, -64.31, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(42.72, -30.06))
                .build();


        TrajectoryFollowerCommand autonomous = new TrajectoryFollowerCommand(drive, right);

        if(isStopRequested()){
        }

        schedule(autonomous);
    }
}