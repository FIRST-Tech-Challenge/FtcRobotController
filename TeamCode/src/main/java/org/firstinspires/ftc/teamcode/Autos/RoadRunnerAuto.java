package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        bot.setPoseEstimate(startPose);

        Trajectory trajectory1 = bot.trajectoryBuilder(startPose) //heading = angle in radians (Math.toRadians(n))
                .strafeRight(10) //10 inches
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        bot.followTrajectory(trajectory1);
    }
}