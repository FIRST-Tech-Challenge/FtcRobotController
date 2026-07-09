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

        for (double degrees = RobotGeometry.TEST_IK_TURRET_MIN_DEGREES;
             degrees <= RobotGeometry.TEST_IK_TURRET_MAX_DEGREES;
             degrees += RobotGeometry.TEST_IK_TURRET_STEP_DEGREES) {
            if (candidateWithinTreatmentWindow(target, degrees)) {
                for (double shoulderAngle = RobotGeometry.TEST_IK_SHOULDER_MIN_ANGLE;
                     shoulderAngle <= RobotGeometry.TEST_IK_SHOULDER_MAX_ANGLE;
                     shoulderAngle += RobotGeometry.TEST_IK_SHOULDER_STEP_DEGREES) {
                    candidates.add(evaluateCandidate(target, degrees, shoulderAngle, minY, maxY));
                }
            }
        }

        robot_system.IKSolution selected = selectBestCandidate(target, candidates, minY, maxY);
        selected.candidateCount = candidates.size();
        return selected;
    }

    private robot_system.IKSolution evaluateCandidate(Position target, double turretDegrees, double shoulderAngle, double minY, double maxY) {
        robot_system.IKSolution solution = new robot_system.IKSolution();
        Position turretOffset = RobotGeometry.turretOffset(turretDegrees);

        double wristAngle = Wrist.levelAngleForShoulder(shoulderAngle);
        // wristAngle is the command that keeps the nozzle level. The IK geometry
        // should use the level world-space nozzle projection, not the command angle.
        double wristLength = RobotGeometry.levelWristLength();
        double wristHeight = RobotGeometry.levelWristHeight();
        double shoulderHeight = RobotGeometry.SHOULDER_RADIUS * Math.sin(Math.toRadians(shoulderAngle));
        double shoulderLength = RobotGeometry.SHOULDER_RADIUS * Math.cos(Math.toRadians(shoulderAngle));
        double predictedY = RobotGeometry.turretTargetY(turretDegrees);
        double predictedZ = TURRENT_TO_CAMERA.z + shoulderHeight + wristHeight;
        double targetErrorY = Math.abs(target.y - predictedY);
        double targetErrorZ = Math.abs(target.z - predictedZ);

        // This is the inverse of ForwardKinematics.computeEndEffectorPose().
        // FK: x = -extension - shoulderLength + turretOffset.x + 2 + wristLength
        double extensionLength = -(target.x - TURRENT_TO_CAMERA.x - 10)
                + turretOffset.x
                - shoulderLength
                + wristLength;

        solution.turretDegrees = turretDegrees;
        solution.turretPotTarget = TurretKinematics.potForDegrees(turretDegrees);
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
        solution.targetErrorY = targetErrorY;
        solution.targetErrorZ = targetErrorZ;

        if (shoulderAngle < RobotGeometry.TEST_IK_SHOULDER_MIN_ANGLE
                || shoulderAngle > RobotGeometry.TEST_IK_SHOULDER_MAX_ANGLE) {
            appendIKFailure(solution, "shoulder angle outside test limit");
        }
        if (targetErrorZ > RobotGeometry.TEST_IK_Z_TOLERANCE) {
            appendIKFailure(solution, "height residual too high");
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

    private boolean candidateWithinTreatmentWindow(Position target, double turretDegrees) {
        double turretY = RobotGeometry.turretTargetY(turretDegrees);
        return turretY >= target.y - RobotGeometry.TEST_IK_TREATMENT_Y_WINDOW
                && turretY <= target.y + RobotGeometry.TEST_IK_TREATMENT_Y_WINDOW;
    }

    private double scoreCandidate(robot_system.IKSolution candidate) {
        double reachablePenalty = candidate.reachable ? 0 : 1000000;
        return reachablePenalty
                + (candidate.targetErrorY * 100)
                + (candidate.targetErrorZ * 100)
                + candidate.extensionLength
                + (Math.abs(candidate.shoulderAngle) / 1000)
                + (Math.abs(candidate.wristAngle) / 10000);
    }

    private robot_system.IKSolution selectBestCandidate(Position target, List<robot_system.IKSolution> candidates, double minY, double maxY) {
        if (candidates.isEmpty()) {
            robot_system.IKSolution solution = new robot_system.IKSolution();
            solution.reachable = false;
            solution.turretDegrees = 0;
            solution.turretPotTarget = TurretKinematics.potForDegrees(0);
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
