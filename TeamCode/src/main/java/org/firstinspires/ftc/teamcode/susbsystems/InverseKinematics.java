package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.Constants.EXTENSION_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_TO_CAMERA;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * Current compute-only IK test solver.
 *
 * This class intentionally does not command motors. It only evaluates candidate
 * turret/shoulder/extension/wrist solutions and returns the best result.
 *
 * Wrist policy: the wrist is not allowed to solve vertical reach. IK keeps the
 * nozzle level, solves vertical reach with the shoulder, then asks Wrist for
 * the command angle needed to stay level at that shoulder angle.
 */
class InverseKinematics {
    robot_system.IKSolution solve(Position target) {
        List<robot_system.IKSolution> candidates = new ArrayList<>();
        double minY = target.y - RobotGeometry.TEST_IK_TREATMENT_Y_WINDOW;
        double maxY = target.y + RobotGeometry.TEST_IK_TREATMENT_Y_WINDOW;

        for (int degrees : RobotGeometry.TEST_IK_TURRET_DEGREES) {
            if (candidateWithinTreatmentWindow(target, degrees)) {
                candidates.add(evaluateCandidate(target, degrees, minY, maxY));
            }
        }

        robot_system.IKSolution selected = selectBestCandidate(target, candidates, minY, maxY);
        selected.candidateCount = candidates.size();
        return selected;
    }

    private robot_system.IKSolution evaluateCandidate(Position target, int turretDegrees, double minY, double maxY) {
        robot_system.IKSolution solution = new robot_system.IKSolution();
        Position turretOffset = RobotGeometry.turretOffset(turretDegrees);

        // The nozzle is intended to stay level, so geometry uses the calibrated horizontal wrist pose.
        double wristPoseAngle = RobotGeometry.TEST_IK_LEVEL_WRIST_POSE_ANGLE;
        double wristLength = RobotGeometry.TEST_IK_WRIST_RADIUS * Math.cos(Math.toRadians(wristPoseAngle));
        double wristHeight = RobotGeometry.TEST_IK_WRIST_RADIUS * Math.sin(Math.toRadians(wristPoseAngle));
        double shoulderHeight = target.z - TURRENT_TO_CAMERA.z - wristHeight;
        double shoulderRatio = shoulderHeight / RobotGeometry.SHOULDER_RADIUS;
        double shoulderAngle = 0;
        boolean validShoulderAngle = Math.abs(shoulderRatio) <= 1;

        if (validShoulderAngle) {
            shoulderAngle = Math.toDegrees(Math.asin(shoulderRatio));
        }

        double wristAngle = Wrist.levelAngleForShoulder(shoulderAngle);
        double shoulderLength = validShoulderAngle
                ? RobotGeometry.SHOULDER_RADIUS * Math.cos(Math.toRadians(shoulderAngle))
                : 0;
        double extensionLength = -(target.x - TURRENT_TO_CAMERA.x - 10)
                + turretOffset.x
                - shoulderLength
                - wristLength;

        solution.turretDegrees = turretDegrees;
        solution.shoulderAngle = shoulderAngle;
        solution.extensionLength = extensionLength;
        solution.wristAngle = wristAngle;
        solution.reachable = true;
        solution.failureReason = "OK";
        solution.acceptedYMin = minY;
        solution.acceptedYMax = maxY;
        solution.targetX = target.x;
        solution.targetY = target.y;
        solution.targetZ = target.z;
        solution.turretOffsetX = turretOffset.x;
        solution.turretOffsetY = turretOffset.y;
        solution.wristLength = wristLength;
        solution.wristHeight = wristHeight;
        solution.shoulderHeight = shoulderHeight;
        solution.shoulderLength = shoulderLength;
        solution.rawExtensionLength = extensionLength;

        if (!validShoulderAngle) {
            appendIKFailure(solution, "shoulder angle outside asin range");
        }
        if (shoulderAngle < RobotGeometry.TEST_IK_SHOULDER_MIN_ANGLE
                || shoulderAngle > RobotGeometry.TEST_IK_SHOULDER_MAX_ANGLE) {
            appendIKFailure(solution, "shoulder angle outside test limit");
        }
        if (shoulderHeight < SHOULDER_MIN_HEIGHT || shoulderHeight > SHOULDER_MAX_HEIGHT) {
            appendIKFailure(solution, "shoulder height outside limit");
        }
        if (extensionLength < 0) {
            appendIKFailure(solution, "extension below minimum");
        }
        if (extensionLength > EXTENSION_MAX_POSITION) {
            appendIKFailure(solution, "extension above maximum");
        }

        solution.selectionScore = scoreCandidate(solution);
        if (!isFiniteSolution(solution)) {
            appendIKFailure(solution, "non-finite IK value");
            solution.selectionScore = 1000000;
        }
        return solution;
    }

    private boolean candidateWithinTreatmentWindow(Position target, int turretDegrees) {
        double turretY = RobotGeometry.turretTargetY(turretDegrees);
        return turretY >= target.y - RobotGeometry.TEST_IK_TREATMENT_Y_WINDOW
                && turretY <= target.y + RobotGeometry.TEST_IK_TREATMENT_Y_WINDOW;
    }

    private double scoreCandidate(robot_system.IKSolution candidate) {
        double reachablePenalty = candidate.reachable ? 0 : 1000000;
        return reachablePenalty
                + candidate.extensionLength
                + (Math.abs(candidate.shoulderAngle) / 1000)
                + (Math.abs(candidate.wristAngle) / 10000);
    }

    private robot_system.IKSolution selectBestCandidate(Position target, List<robot_system.IKSolution> candidates, double minY, double maxY) {
        if (candidates.isEmpty()) {
            robot_system.IKSolution solution = new robot_system.IKSolution();
            solution.reachable = false;
            solution.turretDegrees = 0;
            solution.shoulderAngle = 0;
            solution.extensionLength = 0;
            solution.wristAngle = 0;
            solution.failureReason = "no turret position inside treatment Y window";
            solution.acceptedYMin = minY;
            solution.acceptedYMax = maxY;
            solution.targetX = target.x;
            solution.targetY = target.y;
            solution.targetZ = target.z;
            solution.selectionScore = 1000000;
            return solution;
        }

        robot_system.IKSolution best = candidates.get(0);
        for (robot_system.IKSolution candidate : candidates) {
            if (candidate.selectionScore < best.selectionScore) {
                best = candidate;
            }
        }

        sanitizeIKSolution(best);
        return best;
    }

    private boolean isFiniteSolution(robot_system.IKSolution solution) {
        return Double.isFinite(solution.shoulderAngle)
                && Double.isFinite(solution.extensionLength)
                && Double.isFinite(solution.wristAngle)
                && Double.isFinite(solution.selectionScore);
    }

    private void sanitizeIKSolution(robot_system.IKSolution solution) {
        if (!Double.isFinite(solution.shoulderAngle)) {
            solution.shoulderAngle = 0;
        }
        if (!Double.isFinite(solution.extensionLength)) {
            solution.extensionLength = 0;
        }
        if (!Double.isFinite(solution.wristAngle)) {
            solution.wristAngle = 0;
        }
        if (!Double.isFinite(solution.selectionScore)) {
            solution.selectionScore = 1000000;
        }

        solution.shoulderAngle = clampShoulderAngle(solution.shoulderAngle);
        solution.extensionLength = RobotGeometry.clamp(solution.extensionLength, 0, EXTENSION_MAX_POSITION);
    }

    private double clampShoulderAngle(double angle) {
        return RobotGeometry.clamp(
                angle,
                RobotGeometry.TEST_IK_SHOULDER_MIN_ANGLE,
                RobotGeometry.TEST_IK_SHOULDER_MAX_ANGLE);
    }

    private void appendIKFailure(robot_system.IKSolution solution, String reason) {
        solution.reachable = false;
        if (solution.failureReason.equals("OK")) {
            solution.failureReason = reason;
        } else {
            solution.failureReason += "; " + reason;
        }
    }
}
