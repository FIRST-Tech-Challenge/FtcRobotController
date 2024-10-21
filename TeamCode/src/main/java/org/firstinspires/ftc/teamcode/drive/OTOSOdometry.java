package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

public class OTOSOdometry extends Odometry {
    SparkFunOTOSDrive otos;
    public OTOSOdometry(SparkFunOTOSDrive o) {
        super(new Pose2d(0, 0, new Rotation2d(0)));
        otos = o;
    }
    @Override
    public void updatePose(Pose2d newPose) {
        otos.otos.setPosition(new SparkFunOTOS.Pose2D(newPose.getX(), newPose.getY(), newPose.getHeading()));
        robotPose = newPose;
    }

    @Override
    public void updatePose() {
        update();
    }

    public void update() {
        otos.updatePoseEstimate();
        robotPose = new Pose2d(otos.pose.position.x, otos.pose.position.y, new Rotation2d(otos.pose.heading.log()));
    }
}
