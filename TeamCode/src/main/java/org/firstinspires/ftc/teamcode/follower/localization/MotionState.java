/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.localization;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Twist;
import org.firstinspires.ftc.teamcode.follower.math.Velocity;

public class MotionState {
    private static final MotionState zero = new MotionState(Pose.zero(), Velocity.zero(), Twist.zero());
    private final Pose pose;
    private final Velocity velocity;
    private final Twist twist;

    private MotionState(Pose pose, Velocity velocity, Twist twist) {
        this.pose = pose;
        this.velocity = velocity;
        this.twist = twist;
    }

    public static MotionState zero() {
        return zero;
    }

    public static MotionState ofVelocity(Pose pose, Velocity velocity) {
        return new MotionState(pose, velocity, velocity.toTwist(pose.heading()));
    }

    public static MotionState ofTwist(Pose pose, Twist twist) {
        return new MotionState(pose, twist.toVelocity(pose.heading()), twist);
    }

    public MotionState withPose(Pose pose) {
        return new MotionState(pose, velocity, velocity.toTwist(pose.heading()));
    }

    public Pose pose() {
        return pose;
    }

    public Velocity velocity() {
        return velocity;
    }

    public Twist twist() {
        return twist;
    }

    @Override
    public String toString() {
        return "MotionState{" + "pose=" + pose + ", velocity=" + velocity + ", twist=" + twist + '}';
    }
}
