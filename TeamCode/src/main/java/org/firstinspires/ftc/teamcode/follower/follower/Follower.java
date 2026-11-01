/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.follower;

import org.firstinspires.ftc.teamcode.follower.algorithm.Algorithm;
import org.firstinspires.ftc.teamcode.follower.config.ConfigVar;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.follower.localization.Localizer;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Twist;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.math.Velocity;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
import org.firstinspires.ftc.teamcode.follower.paths.PathSegment;
import org.firstinspires.ftc.teamcode.follower.paths.PathTracker;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Curve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class Follower {
    public final Localizer localizer;
    public final Drivetrain drivetrain;
    public final ConfigVar<Boolean> holdEnd = ConfigVar.of(true);
    private Algorithm algorithm;
    private PathTracker pathTracker = null;
    private Pose holdPose = null;
    private DrivePowers manualPowers = null;
    private Mode mode = Mode.IDLE;
    private long previousNanoTime = 0L;
    private boolean useHoldScaling;
    private final List<Consumer<FollowerLog>> loggers = new ArrayList<>();
    private FollowerLog debug;

    public Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm) {
        this.localizer = localizer;
        this.algorithm = algorithm;
        this.drivetrain = drivetrain;
    }

    /**
     * Adds a logger that will be called every update with the debug information.
     */
    public Follower withLogger(Consumer<FollowerLog> logger) {
        loggers.add(logger);
        return this;
    }

    public Mode mode() {
        return mode;
    }

    public void update() {
        long nanoTime = System.nanoTime();
        double deltaTime = 0;
        if (previousNanoTime != 0L) deltaTime = (nanoTime - previousNanoTime) / 1e9;
        update(deltaTime);
        previousNanoTime = nanoTime;
    }

    public void update(double deltaTime) {
        debug = null;

        localizer.update();

        switch (mode) {
            case FOLLOW: {
                if (pathTracker.done()) {
                    if (holdEnd.get()) {
                        hold(pathTracker.endPose());
                    } else {
                        stop();
                    }
                    pathTracker = null;
                    break;
                }

                DrivePowers powers = algorithm.calculatePath(drivetrain, pathTracker, localizer.state(), deltaTime);
                drivetrain.drive(powers, false);
                break;
            }
            case HOLD: {
                DrivePowers powers =
                        algorithm.calculateHold(drivetrain, holdPose, localizer.state(), useHoldScaling, deltaTime);
                drivetrain.drive(powers, false);
                break;
            }
            case MANUAL: {
                drivetrain.drive(manualPowers, true);
                break;
            }
            case IDLE: {
                drivetrain.stop();
                break;
            }
        }

        if (debug != null && !loggers.isEmpty()) {
            createDebug();
        }

        for (Consumer<FollowerLog> logger : loggers) {
            logger.accept(debug());
        }
    }

    public FollowerLog debug() {
        if (debug == null) createDebug();
        return debug;
    }

    public void createDebug() {
        Map<String, Object> map = new HashMap<>();

        map.put("mode", mode);

        if (mode == Mode.FOLLOW || mode == Mode.HOLD) {
            map.put("isBusy", isBusy());
            map.put("atParametricEnd", atParametricEnd());
            map.put("pathIndex", pathIndex());
        }

        debug = FollowerLog.of(map, localizer.debug(), drivetrain.debug(), algorithm.debug());
    }

    private void clearState() {
        if (pathTracker != null) pathTracker.release();
        pathTracker = null;
        holdPose = null;
        manualPowers = null;
        useHoldScaling = true;
    }

    public void follow(Path path) {
        clearState();
        mode = Mode.FOLLOW;
        pathTracker = new PathTracker(path);
        algorithm.reset();
    }

    public void hold(Pose pose) {
        hold(pose, false);
    }

    public void hold(Pose pose, boolean useScaling) {
        clearState();
        mode = Mode.HOLD;
        holdPose = pose;
        useHoldScaling = useScaling;
    }

    public void manual(DrivePowers powers) {
        clearState();
        mode = Mode.MANUAL;
        manualPowers = powers;
    }

    public void manual(double forward, double lateral, double heading) {
        manual(new DrivePowers(forward, lateral, heading));
    }

    public void stop() {
        clearState();
        mode = Mode.IDLE;
    }

    public void setPose(Pose pose) {
        localizer.setPose(pose);
    }

    public void setX(double x) {
        localizer.setX(x);
    }

    public void setY(double y) {
        localizer.setY(y);
    }

    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    public Pose pose() {
        return localizer.pose();
    }

    public Velocity velocity() {
        return localizer.velocity();
    }

    public Twist twist() {
        return localizer.twist();
    }

    public double distanceToEndpoint() {
        if (pathTracker == null) return 0.0;
        return currentPath().endPose().distance(pose());
    }

    public boolean following() {
        return mode == Mode.FOLLOW;
    }

    public boolean isBusy() {
        return mode != Mode.IDLE && mode != Mode.MANUAL && algorithm.isBusy();
    }

    public boolean holding() {
        return mode == Mode.HOLD;
    }

    public boolean manual() {
        return mode == Mode.MANUAL;
    }

    public boolean idle() {
        return mode == Mode.IDLE;
    }

    public enum Mode {
        FOLLOW,
        HOLD,
        MANUAL,
        IDLE
    }

    public void setAlgorithm(Algorithm algorithm) {
        this.algorithm = algorithm;
    }

    public boolean atParametricEnd() {
        if (!mode.equals(Mode.FOLLOW)) return true;
        if (pathTracker == null) return true;
        if (pathTracker.remainingPaths() > 1) return false;
        return algorithm.atParametricEnd();
    }

    public double completion() {
        return algorithm.completion();
    }

    public Vector2D closestTangent() {
        return algorithm.closestTangent();
    }

    public Vector2D closestNormal() {
        return algorithm.closestNormal();
    }

    public double curvature() {
        return algorithm.curvature();
    }

    public Pose closestPose() {
        return algorithm.closestPose();
    }

    public double parametricCompletion() {
        return algorithm.parametricCompletion();
    }

    public double remainingDistance() {
        return algorithm.remainingDistance();
    }

    public int pathIndex() {
        if (pathTracker == null) return -1;
        return pathTracker.currentIndex();
    }

    public PathSegment currentSegment() {
        if (pathTracker == null) return null;
        return pathTracker.current();
    }

    public Curve currentCurve() {
        if (pathTracker == null) return null;
        return currentSegment().curve;
    }

    public Path currentPath() {
        if (pathTracker == null) return null;
        return pathTracker.path();
    }

    public double tangentialVelocity() {
        return velocity().toVector2D().dot(closestTangent());
    }

    public Algorithm algorithm() {
        return algorithm;
    }

    public Pose poseAt(double completion) {
        if (pathTracker == null) return holdPose == null ? pose() : holdPose;
        return currentSegment().get(currentCurve().parameter(completion));
    }

    public Vector2D tangentAt(double completion) {
        if (pathTracker == null) return closestTangent();
        return currentCurve().tangent(currentCurve().parameter(completion));
    }

    public Vector2D normalAt(double completion) {
        if (pathTracker == null) return closestNormal();
        return currentCurve().leftNormal(currentCurve().parameter(completion));
    }

    public double curvatureAt(double completion) {
        if (pathTracker == null) return curvature();
        return currentCurve().curvature(currentCurve().parameter(completion));
    }
}
