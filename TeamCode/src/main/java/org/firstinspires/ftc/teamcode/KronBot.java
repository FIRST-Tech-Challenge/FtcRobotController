package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.DriveController;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightController;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public final class KronBot implements NextRobot {
    public static final boolean INTAKE_ENABLED = false;

    public final DriveController drive = new DriveController(this);
    public final IntakeController intake = INTAKE_ENABLED ? new IntakeController() : null;
    public final LimelightController vision = new LimelightController();
    public Follower follower;
    private final Set<Mechanism> mechanisms;

    public KronBot() {
        Set<Mechanism> mechanisms = new LinkedHashSet<>();
        mechanisms.add(drive);
        mechanisms.add(vision);
        if (intake != null) {
            mechanisms.add(intake);
        }
        this.mechanisms = Collections.unmodifiableSet(mechanisms);
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    public void initialize(HardwareMap hardwareMap, Pose startingPose) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants
                .createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        drive.initialize();
        if (intake != null) {
            intake.initialize(hardwareMap);
        }
    }

    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void stop() {
        try {
            drive.stop();
        } finally {
            if (intake != null) {
                intake.turnOff();
            }
        }
    }

    public void release() {
        try {
            stop();
        } finally {
            follower = null;
            vision.stop();
        }
    }
}
