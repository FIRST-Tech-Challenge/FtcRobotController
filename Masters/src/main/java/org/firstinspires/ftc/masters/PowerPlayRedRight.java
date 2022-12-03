package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.MultipleCameraCV;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Power Play Red Right")
public class PowerPlayRedRight extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,  telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(36, -60), Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();

        drive.liftTop();
        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(36, -60),Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d( new Vector2d(12,-48), Math.toRadians(90)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(new Vector2d(12,-36),Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(new Vector2d(12,-12),Math.toRadians(135)))
                .build();

        //use vision to align
        //drop cone
        drive.openClaw();

        //park in the correct spot
        //put lift down
    }

}
