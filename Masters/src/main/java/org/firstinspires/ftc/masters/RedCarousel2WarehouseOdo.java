package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Red - Carousel 2 Warehouse (STATE)", group = "competition")
public class RedCarousel2WarehouseOdo extends RedCarouselOdo {

    protected void gotToWarehouse(){

        TrajectorySequence fromHubToWaitPos = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineToSplineHeading(new Pose2d(new Vector2d(5, -60), Math.toRadians(180)))
                .build();
        TrajectorySequence fromWaitPosToWarehouse = drive.trajectorySequenceBuilder(fromHubToWaitPos.end())
                .splineToLinearHeading(new Pose2d( new Vector2d(48, -66), Math.toRadians(180)), Math.toRadians(2))
                .build();

        double seconds = elapsedTime.seconds();
        while (seconds<26) {
            seconds = elapsedTime.seconds();
        }
        drive.followTrajectorySequence(fromWaitPosToWarehouse);
        drive.getCube();
    }


}