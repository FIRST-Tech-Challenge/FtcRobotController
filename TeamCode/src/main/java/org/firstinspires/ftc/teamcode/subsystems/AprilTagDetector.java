package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmode.AprilTag;

public class AprilTagDetector implements Subsystem {
    AprilTag tagOp;
    Pose2d pose2d;
    enum idpositions{
        id1,id2,id3,id4,id5;
    }
    public AprilTagDetector(AprilTag tagOp) {
        this.tagOp = tagOp;
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();
    }
    public Pose2d getTagPose(){
        return tagOp.telemetryAprilTag(pose2d);
    }
}
