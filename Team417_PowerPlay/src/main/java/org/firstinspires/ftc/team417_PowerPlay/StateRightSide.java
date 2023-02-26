package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;

@Autonomous (name="State right")
public class StateRightSide extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto();

        Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        Trajectory pushSignalSleeve = drive.trajectoryBuilder(startPose, false)
                .splineToConstantHeading(new Vector2d(-36, 10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36, 45), Math.toRadians(-90))
                .build();

        Trajectory moveToMidJunction = drive.trajectoryBuilder(pushSignalSleeve.end(), false)
                .splineToConstantHeading(new Vector2d(-24, 36), Math.toRadians(0))
                .build();
        Trajectory moveToCupStack = drive.trajectoryBuilder(moveToMidJunction.end(),false)
                .splineToConstantHeading(new Vector2d(-36,20), Math.toRadians(-90))
                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                .build();

        telemetry.addLine("ready for start");
        telemetry.update();
        waitForStart();

        drive.followTrajectory(pushSignalSleeve);
        drive.followTrajectoryAsync(moveToMidJunction);

        while (opModeIsActive() && drive.isBusy()) {
            telemetry.addLine("is busy working");
            telemetry.update();
            if ((Math.abs(motorArm.getCurrentPosition() - (MID_JUNCT_ARM_POSITION)) > 10)) {
                telemetry.addLine("moving arm");
                telemetry.update();
                motorArm.setPower((MID_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * 1.0/800.0);

            } else {
                motorArm.setPower(HOLD_ARM_AT_MID_OR_LOW_POS_POWER);
            }
            drive.update();
        }

        while (motorArm.getCurrentPosition() > 10) {
            motorArm.setPower(-0.2);//lower arm manual power
        }
        motorArm.setPower(0);
    }
}