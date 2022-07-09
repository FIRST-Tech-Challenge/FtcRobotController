package org.firstinspires.ftc.Team19567.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Team19567.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;

@Autonomous(name="Red Warehouse Modified", group="Dababy")
@Disabled
@Deprecated

//Defensive opmode to block Team 7172 (was never used for obvious reasons)
public class RedWarehouseModified extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MecanumDriveCancelable chassis = new MecanumDriveCancelable(hardwareMap);

        chassis.setPoseEstimate(new Pose2d(10, -63, Math.toRadians(90)));

        TrajectorySequence modifiedSequence = chassis.trajectorySequenceBuilder(new Pose2d(10,-63, Math.toRadians(90)))
                .forward(140).build();

        waitForStart();

        if(!opModeIsActive() || isStopRequested()) return;

        chassis.followTrajectorySequence(modifiedSequence);
    }
}