package org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination;

import com.acmerobotics.roadrunner.Pose2d;

public class Pose {

    public volatile double x, y, theta;

    public Pose (Pose pose) {
        this.x = pose.x;
        this.y = pose.y;
        this.theta = pose.theta;
    }

    public Pose (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose (Vector vector, double theta) {
        setVec(vector);
        this.theta = theta;
    }

    public void setVec(Vector vector) {
        this.x = vector.x;
        this.y = vector.y;
    }

    public void pose2dTranslation(Pose2d pose2d) {
        this.x = pose2d.position.x;
        this.y = pose2d.position.y;
        this.theta = Math.toDegrees(pose2d.heading.toDouble());
    }

    public Vector getVec() {
        return new Vector(x, y);
    }
}
