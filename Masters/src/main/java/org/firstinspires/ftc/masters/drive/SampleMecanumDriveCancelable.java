package org.firstinspires.ftc.masters.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceRunnerCancelable;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ACCEL_TELE;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_VEL_TELE;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_VEL_TELE;
import static org.firstinspires.ftc.masters.drive.DriveConstants.TRACK_WIDTH;

/*
 * Trajectory-cancelable version of the simple mecanum drive hardware implementation for REV hardware.
 * Ensure that this is copied into your project.
 */
@Config
public class SampleMecanumDriveCancelable extends SampleMecanumDrive {

    private TrajectorySequenceRunnerCancelable trajectorySequenceRunnerCancelable;


    public SampleMecanumDriveCancelable(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        super(hardwareMap, opMode, telemetry);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.3);
        VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL_TELE, MAX_ANG_VEL_TELE, TRACK_WIDTH);
        ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL_TELE);

        trajectorySequenceRunnerCancelable = new TrajectorySequenceRunnerCancelable(follower, HEADING_PID);
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunnerCancelable.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }


    public void followTrajectoryAsync(Trajectory trajectory) {
        telemetry.addData("following", "1");
        telemetry.update();
        trajectorySequenceRunnerCancelable.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }


    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunnerCancelable.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void breakFollowing() {
        trajectorySequenceRunnerCancelable.breakFollowing();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunnerCancelable.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunnerCancelable.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public boolean isBusy() {
        return trajectorySequenceRunnerCancelable.isBusy();
    }

}
