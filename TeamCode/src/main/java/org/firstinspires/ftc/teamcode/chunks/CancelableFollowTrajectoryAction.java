package org.firstinspires.ftc.teamcode.chunks;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public class CancelableFollowTrajectoryAction implements Action {
    private final MecanumDrive.FollowTrajectoryAction action;
    private boolean cancelled = false;

    public CancelableFollowTrajectoryAction(MecanumDrive.FollowTrajectoryAction action) {
        this.action = action;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return false;
        }

        return action.run(telemetryPacket);
    }

    private void setDrivePowers(PoseVelocity2d poseVelocity2d) {
    }

    public void cancelAbruptly() {
        cancelled = true;
    }
}
