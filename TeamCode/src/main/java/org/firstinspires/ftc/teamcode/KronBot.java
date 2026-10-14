package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.DriveController;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeController;

/** Robot container based on FTC SDK + Pedro Pathing, without NextFTC. */
public final class KronBot {
    // This matches the last complete pre-NextFTC FTC_TestRobot snapshot.
    public static final boolean INTAKE_ENABLED = false;

    public final DriveController drive = new DriveController(this);
    public final IntakeController intake = INTAKE_ENABLED ? new IntakeController() : null;
    public Follower follower;

    public void initialize(HardwareMap hardwareMap, Pose startingPose) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants
                .createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        drive.initialize();
        if (intake != null) intake.initialize(hardwareMap);
    }

    public void update() {
        if (follower != null) follower.update();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void stop() {
        try {
            drive.stop();
        } finally {
            if (intake != null) intake.turnOff();
        }
    }

    public void release() {
        stop();
        follower = null;
    }
}
