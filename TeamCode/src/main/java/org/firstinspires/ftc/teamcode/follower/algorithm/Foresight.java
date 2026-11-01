/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.algorithm;

import static org.firstinspires.ftc.teamcode.follower.utils.Angle.normalizeSigned;
import static org.firstinspires.ftc.teamcode.follower.utils.Angle.turnDirection;

import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.follower.localization.MotionState;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Twist;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.PathTracker;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Curve;
import org.firstinspires.ftc.teamcode.follower.utils.Pair;
import org.firstinspires.ftc.teamcode.follower.utils.Timer;
import org.firstinspires.ftc.teamcode.follower.utils.Utils;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class Foresight implements Algorithm {
    public final ForesightConfig config;
    public final ForesightPowerAllocator allocator;
    private double closestT, curvature;
    private double projectedClosestT, coastClosestT;
    private double curveCompletion, remainingDistance, tangentialSpeed;
    private Pose closestPose;
    private Vector2D closestTangent, closestNormal;
    private final Timer timer = new Timer();
    private boolean resetTimer = true;
    private double headingError, translationalError, targetVelocity;
    private boolean busy = false;
    private boolean isBraking = false;
    private final Vector2D naturalDeceleration;
    private PathTracker tracker;
    private MotionState currentState;
    private Map<String, Object> debug = new HashMap<>();

    public Foresight(ForesightConfig config) {
        this.config = config;
        this.allocator = new ForesightPowerAllocator(config);
        this.naturalDeceleration =
                Vector2D.cartesian(config.naturalForwardDeceleration.get(), config.naturalStrafeDeceleration.get());
    }

    @Override
    public DrivePowers calculatePath(
            Drivetrain drivetrain, PathTracker pathTracker, MotionState state, double deltaTime) {
        tracker = pathTracker;
        currentState = state;
        closestT = pathTracker.current().curve.closestParameter(state.pose().toVector2D(), closestT);

        if (parametricCondition()) { // End Constraint
            closestT = 1.0;
            double targetHeading = pathTracker.current().heading(closestT);
            closestPose = pathTracker.current().curve.get(closestT).toPose(targetHeading);

            if (pathTracker.remainingPaths() > 1) { // advance if constraints met
                pathTracker.advance();
                reset();
                return calculatePath(drivetrain, pathTracker, state, deltaTime);
            }

            pathTracker.advance();
            reset();
            return DrivePowers.zero();
        } else {
            // TODO: Cache these instead so only computed on user request
            double targetHeading = pathTracker.current().heading(closestT);
            closestPose = pathTracker.current().curve.get(closestT).toPose(targetHeading);
            curvature = pathTracker.current().curve.curvature(closestT);
            remainingDistance = pathTracker.current().curve.remainingDistance(closestT);
            curveCompletion =
                    1 - remainingDistance / pathTracker.current().curve.length();
            closestTangent = pathTracker.current().curve.tangent(closestT);
            closestNormal = pathTracker.current().curve.leftNormal(closestT);
            tangentialSpeed = state.velocity().toVector2D().dot(closestTangent);
            translationalError = state.pose().distance(closestPose);
            this.headingError = normalizeSigned(targetHeading - state.pose().heading());
        }

        Curve curve = pathTracker.current().curve;
        Pose projectedPose = state.pose()
                .plus(getBrakeDisplacement(state.twist(), state.pose().heading()));
        projectedClosestT = curve.closestParameter(projectedPose.toVector2D(), projectedClosestT);
        double targetHeading = closestPose.heading();
        double projectedTargetHeading = pathTracker.current().heading(projectedClosestT);
        Vector2D projectedTangent = curve.tangent(projectedClosestT);
        Vector2D projectedTargetPos = curve.get(projectedClosestT);
        Vector2D projectedNormal = curve.leftNormal(projectedClosestT);
        double projectedRemainingDist = curve.remainingDistance(projectedClosestT);
        if (projectedRemainingDist <= 0.01)
            projectedRemainingDist = projectedTangent.dot(projectedTargetPos.minus(projectedPose.toVector2D()));
        double angleToTangent = projectedTangent.theta() - projectedPose.heading();

        double velocityToBrakeInTime =
                getVelocityToBrakeInTime(projectedRemainingDist, projectedTangent, projectedPose.heading());
        isBraking = velocityToBrakeInTime <= 0 || projectedRemainingDist <= 0;

        if (isBraking && pathTracker.remainingPaths() > 1 && config.pathSkip.get()) {
            pathTracker.advance();
            reset();
            return calculatePath(drivetrain, pathTracker, state, deltaTime);
        }

        double headingError = normalizeSigned(projectedTargetHeading - projectedPose.heading());
        double currentHeadingError =
                normalizeSigned(targetHeading - state.pose().heading());

        double totalHeadingPower = config.headingFeedback.get().calculate(0, headingError);
        double headingFeedbackPower = config.headingFeedback.get().calculate(0, currentHeadingError);
        double headingFeedforwardPower;

        if (Math.abs(totalHeadingPower) <= 1e-3) {
            headingFeedbackPower = 0;
            headingFeedforwardPower = 0;
        } else if (totalHeadingPower * headingFeedbackPower < 0) {
            headingFeedforwardPower = totalHeadingPower;
            headingFeedbackPower = 0;
        } else if (Math.abs(headingFeedbackPower) >= Math.abs(totalHeadingPower)) {
            headingFeedforwardPower = 0;
            headingFeedbackPower = totalHeadingPower;
        } else {
            headingFeedforwardPower = totalHeadingPower - headingFeedbackPower;
        }

        headingFeedforwardPower += config.headingStaticFF.get().calculate(0, turnDirection(headingError));

        Vector2D drive = projectedTangent.times(drive(
                isBraking && config.brakeAtEnd.get(),
                velocityToBrakeInTime,
                deltaTime,
                angleToTangent,
                tangentialSpeed,
                state,
                curve,
                drivetrain));

        Pair<Double, Vector2D> translationalResult =
                translationalCorrection(projectedPose, projectedTargetPos, projectedNormal);
        Vector2D translational = translationalResult.second();
        double translationalError = translationalResult.first();

        boolean atParametricStart = projectedClosestT <= config.parametricTConstraint.get();

        if (atParametricStart) {
            Vector2D displacementToStart = curve.startPoint().minus(projectedPose.toVector2D());
            double tangentDisplacementToStart = displacementToStart.dot(closestTangent);
            boolean isBeforePath =
                    tangentDisplacementToStart > 1 && translationalError > config.translationalDeviationTolerance.get();
            boolean isHeadingBeforePath = Math.abs(headingError) > config.headingDeviationTolerance.get();

            if (isBeforePath && config.cosineScale.get())
                drive = drive.times(tangentDisplacementToStart / translationalError);

            if (isHeadingBeforePath && config.cosineScale.get()) {
                drive = drive.times(allocator.getDriveScalar(0, headingError));
            }
        } else {
            if (((Math.abs(headingError) > 2 * config.headingDeviationTolerance.get())
                            || (Math.abs(translationalError) > 2 * config.translationalDeviationTolerance.get()))
                    && config.cosineScale.get())
                drive = drive.times(allocator.getDriveScalar(translationalError, headingError));
        }

        isBraking = isBraking && config.brakeAtEnd.get();

        DrivePowers drivePowers = allocator.allocatePowers(
                drivetrain,
                state,
                Vector2D.zero(),
                headingFeedforwardPower,
                translational,
                drive,
                headingFeedbackPower,
                translationalError,
                headingError);

        debug = debugData();

        return drivePowers;
    }

    @Override
    public DrivePowers calculateHold(
            Drivetrain drivetrain, Pose target, MotionState state, boolean useScaling, double deltaTime) {
        tracker = null;
        currentState = state;
        closestT = 1.0;
        isBraking = false;

        if (resetTimer) {
            timer.reset();
            resetTimer = false;
        }

        closestPose = target;
        Pose projectedPose = state.pose()
                .plus(getBrakeDisplacement(state.twist(), state.pose().heading()));
        double headingCorrection =
                headingFeedback(projectedPose.heading(), target.heading()).second();
        headingError = normalizeSigned(target.heading() - state.pose().heading());

        Vector2D displacement = target.minus(projectedPose).toVector2D();
        double dist = displacement.magnitude();

        translationalError = target.distance(state.pose());
        Vector2D translational = Vector2D.zero();
        if (dist < 1e-3) {
            tangentialSpeed = 0;
            closestTangent = Vector2D.zero();
            closestNormal = closestTangent;
        } else {
            closestTangent = displacement.div(dist);
            tangentialSpeed = closestTangent.dot(state.velocity().toVector2D());

            Pair<Double, Vector2D> translationalResult =
                    translationalCorrection(projectedPose, target.toVector2D(), closestTangent);
            translational = translationalResult.second();
            closestNormal = closestTangent;
        }

        if (busy && timeoutCondition() || (headingCondition() && translationalCondition() && velocityCondition()))
            busy = false;

        if (useScaling) {
            double translationalScale = config.holdPointTranslationalScaling.get();
            double headingScale = config.holdPointHeadingScaling.get();

            translational = translational.times(translationalScale);
            headingCorrection *= headingScale;
        }

        DrivePowers drivePowers = allocator.getDrivePowers(translational, state, headingCorrection);

        debug = debugData();

        return drivePowers;
    }

    public Pose getBrakeDisplacement(Twist twist, double heading) {
        Vector2D linearTwist = twist.toVector2D();
        Vector2D quadratic =
                linearTwist.hadamardProduct(linearTwist.abs()).transform(config.quadraticBrakeCoefficients.get());
        Vector2D linear = linearTwist.transform(config.linearBrakeCoefficients.get());
        double headingDisp = twist.omega
                        * Math.abs(twist.omega)
                        * config.headingBrakeCoefficients.get().y()
                + twist.omega * config.headingBrakeCoefficients.get().x();
        Vector2D bodyDisp = quadratic.plus(linear);
        Pose worldPose = new Pose(0, 0, heading).exp(new Twist(bodyDisp.x(), bodyDisp.y(), headingDisp));
        return new Pose(worldPose.x(), worldPose.y(), headingDisp);
    }

    public Pose getCoastDisplacement(Twist twist, double heading) {
        Vector2D linearTwist = twist.toVector2D();
        Vector2D bodyDisp =
                linearTwist.hadamardProduct(linearTwist.abs()).times(1.0 / 2.0).elementDivision(naturalDeceleration);
        double headingDisp = twist.omega
                        * Math.abs(twist.omega)
                        * config.headingBrakeCoefficients.get().y()
                + twist.omega * config.headingBrakeCoefficients.get().x();
        Pose worldPose = new Pose(0, 0, heading).exp(new Twist(bodyDisp.x(), bodyDisp.y(), headingDisp));
        return new Pose(worldPose.x(), worldPose.y(), headingDisp);
    }

    public double getVelocityToBrakeInTime(double distanceRemaining, Vector2D closestTangent, double heading) {
        Vector2D t = closestTangent.toBodyFrame(heading).abs();
        Vector2D t2 = t.hadamardProduct(t);
        Vector2D t3 = t2.hadamardProduct(t);

        double k1 = config.quadraticBrakeCoefficients.get().get(0, 0) * t3.x()
                + config.quadraticBrakeCoefficients.get().get(1, 1) * t3.y();
        double k2 = config.linearBrakeCoefficients.get().get(0, 0) * t2.x()
                + config.linearBrakeCoefficients.get().get(1, 1) * t2.y();
        Pair<Double, Double> velocityInversion =
                Utils.solveQuadratic(k1, k2, -Math.abs(distanceRemaining) / config.brakeAggression.get());
        return Math.max(velocityInversion.first(), velocityInversion.second()) * Math.signum(distanceRemaining);
    }

    private Pair<Double, Double> headingFeedback(double currentHeading, double targetHeading) {
        double headingError = normalizeSigned(targetHeading - currentHeading);
        return Pair.of(
                headingError,
                config.headingFeedback.get().calculate(0, headingError)
                        + (config.headingStaticFF.get().calculate(0, turnDirection(headingError))));
    }

    public double drive(
            boolean isBraking,
            double profiledTargetVelocity,
            double deltaTime,
            double angleToTangent,
            double tangentialVel,
            MotionState state,
            Curve curve,
            Drivetrain drivetrain) {
        double maxAchievableVelocity = drivetrain.interpolateVelocity(
                config.maxAchievableForwardVelocity.get(), config.maxAchievableStrafeVelocity.get(), angleToTangent);
        targetVelocity = Math.min(profiledTargetVelocity, maxAchievableVelocity);

        if (!isBraking)
            return coast(tangentialVel, deltaTime, maxAchievableVelocity, state, curve, angleToTangent, drivetrain);
        return config.brake.get().calculate(targetVelocity, 0);
    }

    public double coast(
            double tangentialVel,
            double deltaTime,
            double maxAchievableVelocity,
            MotionState state,
            Curve curve,
            double theta,
            Drivetrain drivetrain) {
        double maxAccelerationConstraint = config.maxAccelerationConstraint.get();
        double maxVelocityConstraint = config.maxVelocityConstraint.get();
        double maxDecelerationConstraint = config.maxDecelerationConstraint.get();
        double coastDownToVelocity = config.coastDownToVelocity.get();
        double maxPathSpeed = config.maxPathSpeed.get();
        double maxDecelerationScale = config.maxDecelerationScale.get();

        if (maxDecelerationScale != ForesightConfig.Constraint.NONE) {
            double naturalDeceleration = drivetrain.interpolateAcceleration(
                    config.naturalForwardDeceleration.get(), config.naturalStrafeDeceleration.get(), theta);
            maxDecelerationConstraint = Math.min(maxDecelerationScale * naturalDeceleration, maxDecelerationConstraint);
        }

        double targetVel = maxAchievableVelocity;

        if (maxPathSpeed != ForesightConfig.Constraint.NONE)
            targetVel = Math.min(targetVel, maxPathSpeed * maxAchievableVelocity);

        if (maxVelocityConstraint != ForesightConfig.Constraint.NONE)
            targetVel = Math.min(targetVel, maxVelocityConstraint);

        if (maxAccelerationConstraint != ForesightConfig.Constraint.NONE)
            targetVel = Math.min(targetVel, tangentialVel + maxAccelerationConstraint * deltaTime);

        double feedforwardVelocity = targetVel;

        if (maxDecelerationConstraint != ForesightConfig.Constraint.NONE) {
            Pose projected = state.pose()
                    .plus(getCoastDisplacement(state.twist(), state.pose().heading()));
            double projectedT = curve.closestParameter(projected.toVector2D(), coastClosestT);
            Vector2D projectedTangent = curve.tangent(projectedT);
            double projectedRemainingDist = curve.remainingDistance(projectedT);
            if (projectedRemainingDist <= 0.01)
                projectedRemainingDist =
                        projectedTangent.dot(curve.get(projectedT).minus(projected.toVector2D()));
            double discrim =
                    coastDownToVelocity * coastDownToVelocity + 2 * maxDecelerationConstraint * projectedRemainingDist;
            double excessVel = excessVelAfterCoast(projectedRemainingDist, theta, drivetrain);
            double velocityMomentumCannotProvide = Math.max(0, coastDownToVelocity - excessVel);
            double velocityNeededToCoastInTime = Math.sqrt(Math.abs(discrim)) * Math.signum(discrim);
            targetVel = Math.min(targetVel, velocityNeededToCoastInTime);
            feedforwardVelocity = Math.min(feedforwardVelocity, velocityMomentumCannotProvide);
        }

        if (targetVel >= maxAchievableVelocity) {
            targetVelocity = maxAchievableVelocity;
            return 1;
        }

        double error = Math.max(0, targetVel);
        targetVelocity = targetVel;
        return Math.max(config.coast.get().calculate(feedforwardVelocity, error), 0);
    }

    private Pair<Double, Vector2D> translationalCorrection(
            Pose currentPose, Vector2D targetPos, Vector2D closestNormal) {
        Vector2D displacement = targetPos.minus(currentPose.toVector2D()).projectOnto(closestNormal);
        double translationalError = displacement.magnitude();
        if (translationalError < config.minCorrectionDistance.get()) return Pair.of(0.0, Vector2D.zero());

        Vector2D bodyFrameError = displacement.toBodyFrame(currentPose.heading());
        return Pair.of(
                translationalError,
                Vector2D.cartesian(
                                config.forwardTranslational.get().calculate(0, bodyFrameError.x()),
                                config.strafeTranslational.get().calculate(0, bodyFrameError.y()))
                        .projectOnto(bodyFrameError)
                        .toWorldFrame(currentPose.heading()));
    }

    private double excessVelAfterCoast(double remainingDistance, double theta, Drivetrain drivetrain) {
        double naturalDeceleration = drivetrain.interpolateAcceleration(
                config.naturalForwardDeceleration.get(), config.naturalStrafeDeceleration.get(), theta);
        double excessVelocitySquared = -2 * naturalDeceleration * remainingDistance;
        return Math.signum(excessVelocitySquared) * Math.sqrt(Math.abs(excessVelocitySquared));
    }

    public double headingError() {
        return headingError;
    }

    public double translationalError() {
        return translationalError;
    }

    public boolean velocityCondition() {
        return Math.abs(tangentialSpeed) < config.velocityConstraint.get();
    }

    public boolean translationalCondition() {
        return Math.abs(translationalError) < config.translationalConstraint.get();
    }

    public boolean headingCondition() {
        return Math.abs(headingError) < config.headingConstraint.get();
    }

    public boolean parametricCondition() {
        return closestT >= (1 - config.parametricTConstraint.get());
    }

    public boolean timeoutCondition() {
        return !resetTimer && timer.get(TimeUnit.MILLISECONDS) > config.timeoutConstraint.get();
    }

    public double targetVelocity() {
        return targetVelocity;
    }

    @Override
    public double completion() {
        if (tracker == null) return 1.0;
        return tracker.path().curve.pathCompletion(closestT);
    }

    @Override
    public Pose closestPose() {
        return closestPose;
    }

    @Override
    public Vector2D closestTangent() {
        return closestTangent;
    }

    @Override
    public Vector2D closestNormal() {
        return closestNormal;
    }

    @Override
    public double curvature() {
        return curvature;
    }

    @Override
    public double remainingDistance() {
        return remainingDistance;
    }

    @Override
    public double parametricCompletion() {
        return closestT;
    }

    @Override
    public boolean atParametricEnd() {
        return parametricCondition();
    }

    @Override
    public void reset() {
        timer.reset();
        resetTimer = true;
        busy = true;
        closestT = 0.0;
        projectedClosestT = 0.0;
        coastClosestT = 0.0;
        tracker = null;
        isBraking = false;
        config.forwardTranslational.get().reset();
        config.strafeTranslational.get().reset();
        config.headingFeedback.get().reset();
        config.headingStaticFF.get().reset();
        config.brake.get().reset();
        config.coast.get().reset();
    }

    @Override
    public boolean isBusy() {
        return busy;
    }

    public Map<String, Object> debugData() {
        Map<String, Object> map = new HashMap<>();

        map.put("pose", currentState.pose());
        map.put("velocity", currentState.velocity());
        map.put("twist", currentState.twist());
        map.put("translationalError", translationalError);
        map.put("headingError", headingError);
        map.put("tangentialSpeed", tangentialSpeed);
        map.put("closestT", closestT);
        map.put("projectedClosestT", projectedClosestT);
        map.put("remainingDistance", remainingDistance);
        map.put("curveCompletion", curveCompletion);
        map.put("normalFeedforward", allocator.getNormalFeedforwardVector());
        map.put("headingFeedforward", allocator.getHeadingFeedforward());
        map.put("translationalVector", allocator.getTranslationalVector());
        map.put("driveVector", allocator.getDriveVector());
        map.put("headingPower", allocator.getHeadingPower());

        return map;
    }

    public boolean isBraking() {
        return isBraking;
    }

    @Override
    public Map<String, Object> debug() {
        return debug;
    }
}
