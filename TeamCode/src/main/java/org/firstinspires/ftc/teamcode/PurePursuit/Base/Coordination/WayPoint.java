package org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination;

import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxAcceleration;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxRadiusRange;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxRotationalAcceleration;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxRotationalVelocity;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxVelocity;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.minRadiusRange;

public class WayPoint {

    public enum WaypointType{END, DEFAULT}

    public Pose pose;
    public double[] threshold = new double[2];
    public double
        min_radius = minRadiusRange,
        max_radius = maxRadiusRange,
        max_vel = maxVelocity,
        max_accel = maxAcceleration,
        max_rot_vel = maxRotationalVelocity,
        max_rot_accel = maxRotationalAcceleration;

    public WaypointType type;

    public static class WaypointBuilder {
        private double
            min_radius, max_radius,
            max_vel, max_accel, max_rot_vel, max_rot_accel;

        private double[] threshold = new double[2];
        private Pose pose;

        private WaypointType type;

        public WaypointBuilder(Pose pose, WaypointType type) {
            this.pose = pose;
            this.type = type;
        }

        public WaypointBuilder threshold(double[] threshold) {
            this.threshold = threshold;
            return this;
        }

        public WaypointBuilder min_radius(double min_radius) {
            this.min_radius = min_radius;
            return this;
        }

        public WaypointBuilder max_radius(double max_radius) {
            this.max_radius = max_radius;
            return this;
        }

        public WaypointBuilder max_vel(double max_vel) {
            this.max_vel = max_vel;
            return this;
        }

        public WaypointBuilder max_accel(double max_accel) {
            this.max_accel = max_accel;
            return this;
        }

        public WaypointBuilder max_rot_vel(double max_rot_vel) {
            this.max_rot_vel = max_rot_vel;
            return this;
        }

        public WaypointBuilder max_rot_accel(double max_rot_accel) {
            this.max_rot_accel = max_rot_accel;
            return this;
        }

        public WayPoint build() {
            return new WayPoint(this);
        }
    }

    private WayPoint(WaypointBuilder builder) {
        this.pose = builder.pose;
        this.type = builder.type;
        this.threshold = builder.threshold;
        this.min_radius = builder.min_radius;
        this.max_radius = builder.max_radius;
        this.max_vel = builder.max_vel;
        this.max_accel = builder.max_accel;
        this.max_rot_vel = builder.max_rot_vel;
        this.max_rot_accel = builder.max_rot_accel;
    }

    public WayPoint(WayPoint point) {
        pose = point.pose;
        type = point.type;
        threshold = point.threshold;
        min_radius = point.min_radius;
        max_radius = point.max_radius;
        max_vel = point.max_vel;
        max_accel = point.max_accel;
        max_rot_vel = point.max_rot_vel;
        max_rot_accel = point.max_rot_accel;
    }

    public void setPoint (Vector vector) {
        pose.setVec(vector);
    }
}
