package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class ArchytasMovementAlgorithm {
    // Main Wolfpack Movement Algorithm
        private HybridOdometry odometry;
        private MecanumDriveCalibration mecanumCalibration;
        private PIDController pathDistancePid;
        private PIDController headingPid;
        private PIDController xPid, yPid; // For point-to-point movement

        // Path following
        private BezierCurve currentPath;
        private double currentT;
        private Point targetPoint;
        private static final double T_INCREMENT = 0.005; // Smaller increment for better precision

        // Power hierarchy constants
        private static final double MAX_CORRECTIVE_POWER = 0.8;
        private static final double MAX_HEADING_POWER = 0.6;
        private static final double MAX_DRIVE_POWER = 1.0;

        // Teleop enhancement
        private Queue<Point> positionHistory;
        private Queue<RobotState> stateHistory;
        private static final int MAX_HISTORY_SIZE = 10;

        // Velocity-based stopping
        private double measuredDeceleration = 2.0; // units/sec^2
        private static final double POSITION_TOLERANCE = 2.0; // inches
        private static final double HEADING_TOLERANCE = Math.toRadians(2); // 2 degrees

        // State tracking
        private boolean pathComplete = false;
        private double lastUpdateTime = 0;

        public ArchytasMovementAlgorithm() {
            odometry = new HybridOdometry(0.001);
            mecanumCalibration = new MecanumDriveCalibration();

            // Tuned PID controllers
            pathDistancePid = new PIDController(0.8, 0.0, 0.1, 1.0, MAX_CORRECTIVE_POWER);
            headingPid = new PIDController(1.2, 0.0, 0.05, 1.0, MAX_HEADING_POWER);
            xPid = new PIDController(0.6, 0.0, 0.08, 1.0, MAX_DRIVE_POWER);
            yPid = new PIDController(0.6, 0.0, 0.08, 1.0, MAX_DRIVE_POWER);

            currentPath = null;
            currentT = 0.0;
            targetPoint = null;

            positionHistory = new LinkedList<>();
            stateHistory = new LinkedList<>();
        }

        public void setPath(List<Point> controlPoints) {
            if (controlPoints.size() < 2) {
                throw new IllegalArgumentException("Path must have at least 2 control points");
            }
            currentPath = new BezierCurve(controlPoints);
            currentT = 0.0;
            pathComplete = false;
            targetPoint = controlPoints.get(controlPoints.size() - 1);
        }

        public void setTarget(Point target) {
            targetPoint = target.copy();
            currentPath = null;
            pathComplete = false;
        }

        private double findClosestPointOnPath(Point robotPos) {
            if (currentPath == null) return 0.0;

            double minDistance = Double.MAX_VALUE;
            double bestT = currentT;

            // Search forward from current T
            double searchStart = Math.max(0, currentT - 0.1);
            double searchEnd = Math.min(1.0, currentT + 0.2);

            for (double searchT = searchStart; searchT <= searchEnd; searchT += T_INCREMENT) {
                Point pathPoint = currentPath.evaluate(searchT);
                double distance = robotPos.distanceTo(pathPoint);

                if (distance < minDistance) {
                    minDistance = distance;
                    bestT = searchT;
                }
            }

            return bestT;
        }

        private Point calculateLookaheadPoint(double baseT, double lookaheadDistance) {
            if (currentPath == null) return null;

            Point basePoint = currentPath.evaluate(baseT);

            // Binary search for lookahead point
            double lowT = baseT;
            double highT = 1.0;
            double targetT = baseT;

            while (highT - lowT > T_INCREMENT) {
                double midT = (lowT + highT) / 2.0;
                Point midPoint = currentPath.evaluate(midT);
                double distance = basePoint.distanceTo(midPoint);

                if (distance < lookaheadDistance) {
                    lowT = midT;
                    targetT = midT;
                } else {
                    highT = midT;
                }
            }

            return currentPath.evaluate(targetT);
        }

        public double[] autonomousPathFollowing(RobotState robotState, double dt) {
            if (currentPath == null) return new double[]{0, 0, 0, 0};

            Point robotPos = robotState.getPosition();

            // Update current T to closest point on path
            currentT = findClosestPointOnPath(robotPos);

            // Check if we've reached the end
            if (currentT >= 0.95 && robotPos.distanceTo(targetPoint) < POSITION_TOLERANCE) {
                pathComplete = true;
                return new double[]{0, 0, 0, 0};
            }

            // Calculate lookahead point
            double speed = robotState.getSpeed();
            double dynamicLookahead = Math.max(6.0, speed * 0.5); // Minimum 6 inches lookahead
            Point lookaheadPoint = calculateLookaheadPoint(currentT, dynamicLookahead);

            if (lookaheadPoint == null) {
                lookaheadPoint = currentPath.evaluate(Math.min(1.0, currentT + 0.1));
            }

            // Calculate path direction
            Point pathDirection = lookaheadPoint.subtract(robotPos).normalize();

            // Calculate cross-track error
            Point closestPoint = currentPath.evaluate(currentT);
            double crossTrackError = robotPos.distanceTo(closestPoint);

            // Determine which side of the path we're on
            Point pathTangent = currentPath.derivative(currentT).normalize();
            Point toRobot = robotPos.subtract(closestPoint);
            double crossProduct = pathTangent.x * toRobot.y - pathTangent.y * toRobot.x;
            if (crossProduct < 0) crossTrackError *= -1;

            // Calculate corrective power
            double correctivePower = pathDistancePid.calculate(crossTrackError, dt);

            // Calculate perpendicular direction to path for correction
            Point correctionDirection = new Point(-pathTangent.y, pathTangent.x);

            // Calculate desired heading
            double desiredHeading = Math.atan2(pathDirection.y, pathDirection.x);
            double headingError = normalizeAngle(desiredHeading - robotState.heading);
            double headingPower = headingPid.calculate(headingError, dt);

            // Calculate drive powers
            double baseSpeed = Math.min(0.8, Math.max(0.3, 1.0 - Math.abs(headingError)));

            double driveX = baseSpeed * pathDirection.x + correctivePower * correctionDirection.x;
            double driveY = baseSpeed * pathDirection.y + correctivePower * correctionDirection.y;

            // Limit drive power
            double driveMagnitude = Math.sqrt(driveX * driveX + driveY * driveY);
            if (driveMagnitude > MAX_DRIVE_POWER) {
                driveX = driveX / driveMagnitude * MAX_DRIVE_POWER;
                driveY = driveY / driveMagnitude * MAX_DRIVE_POWER;
            }

            return mecanumCalibration.calculateMotorPowers(driveX, driveY, headingPower);
        }

        public double[] autonomousPointToPoint(RobotState robotState, double targetHeading, double dt) {
            if (targetPoint == null) return new double[]{0, 0, 0, 0};

            Point robotPos = robotState.getPosition();
            double distanceToTarget = robotPos.distanceTo(targetPoint);

            // Check if we've reached the target
            if (distanceToTarget < POSITION_TOLERANCE &&
                    Math.abs(normalizeAngle(targetHeading - robotState.heading)) < HEADING_TOLERANCE) {
                pathComplete = true;
                return new double[]{0, 0, 0, 0};
            }

            // Calculate errors
            double xError = targetPoint.x - robotPos.x;
            double yError = targetPoint.y - robotPos.y;
            double headingError = normalizeAngle(targetHeading - robotState.heading);

            // Calculate PID outputs
            double xOutput = xPid.calculate(xError, dt);
            double yOutput = yPid.calculate(yError, dt);
            double headingOutput = headingPid.calculate(headingError, dt);

            // Apply velocity-based deceleration near target
            double decelerationFactor = Math.min(1.0, distanceToTarget / (POSITION_TOLERANCE * 3));
            xOutput *= decelerationFactor;
            yOutput *= decelerationFactor;

            return mecanumCalibration.calculateMotorPowers(xOutput, yOutput, headingOutput, true, robotState.heading);
        }

        public double[] teleopUpdate(RobotState robotState, double joystickX, double joystickY,
                                     double joystickRotation, boolean fieldCentric, double dt) {
            // Update state history
            stateHistory.offer(new RobotState(robotState.x, robotState.y, robotState.heading));
            if (stateHistory.size() > MAX_HISTORY_SIZE) {
                stateHistory.poll();
            }

            // Apply input shaping (cubic scaling for fine control)
            joystickX = Math.signum(joystickX) * Math.pow(Math.abs(joystickX), 1.5);
            joystickY = Math.signum(joystickY) * Math.pow(Math.abs(joystickY), 1.5);
            joystickRotation = Math.signum(joystickRotation) * Math.pow(Math.abs(joystickRotation), 1.5);

            return mecanumCalibration.calculateMotorPowers(joystickX, joystickY, joystickRotation, fieldCentric, robotState.heading);
        }

        private double normalizeAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        // Utility methods for OpMode integration
        public void updateOdometry(double leftEncoder, double rightEncoder, double perpendicularEncoder,
                                   double imuHeading, double dt) {
            odometry.update(leftEncoder, rightEncoder, perpendicularEncoder, imuHeading, dt);
        }

        public RobotState getCurrentState() {
            Point pos = odometry.getPosition();
            RobotState state = new RobotState(pos.x, pos.y, odometry.getHeading());

            // Calculate velocities if we have history
            if (stateHistory.size() >= 2) {
                RobotState[] states = stateHistory.toArray(new RobotState[0]);
                RobotState prev = states[states.length - 2];
                RobotState curr = states[states.length - 1];

                double timeDiff = (curr.timestamp - prev.timestamp) / 1000.0; // Convert to seconds
                if (timeDiff > 0) {
                    state.velocityX = (curr.x - prev.x) / timeDiff;
                    state.velocityY = (curr.y - prev.y) / timeDiff;
                    state.angularVelocity = normalizeAngle(curr.heading - prev.heading) / timeDiff;
                }
            }

            return state;
        }

        public boolean isPathComplete() { return pathComplete; }
        public void resetPath() { pathComplete = false; currentT = 0.0; }

        public void setPIDGains(String controller, double kp, double ki, double kd) {
            switch (controller.toLowerCase()) {
                case "path":
                    pathDistancePid.setGains(kp, ki, kd);
                    break;
                case "heading":
                    headingPid.setGains(kp, ki, kd);
                    break;
                case "x":
                    xPid.setGains(kp, ki, kd);
                    break;
                case "y":
                    yPid.setGains(kp, ki, kd);
                    break;
            }
        }

        public void calibrateMecanum(double forwardVelocity, double sidewaysVelocity) {
            mecanumCalibration.calibrate(forwardVelocity, sidewaysVelocity);
        }

        public void setPosition(Point position) {
            odometry.setPosition(position);
        }

        public void setHeading(double heading) {
            odometry.setHeading(heading);
        }
    }

