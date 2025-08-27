package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;

public class HybridOdometry {
        private static final double ENCODER_TICKS_PER_INCH = 1000; // Configurable
        private static final double TRACK_WIDTH = 15.0; // inches

        private double imuAccuracyThreshold;
        private Point position;
        private double heading;
        private double prevLeftEncoder, prevRightEncoder, prevPerpendicularEncoder;
        private boolean initialized;

        public HybridOdometry(double imuAccuracyThreshold) {
            this.imuAccuracyThreshold = imuAccuracyThreshold;
            this.position = new Point(0, 0);
            this.heading = 0;
            this.prevLeftEncoder = 0;
            this.prevRightEncoder = 0;
            this.prevPerpendicularEncoder = 0;
            this.initialized = false;
        }

        public void update(double leftEncoder, double rightEncoder, double perpendicularEncoder,
                           double imuHeading, double dt) {
            if (!initialized) {
                prevLeftEncoder = leftEncoder;
                prevRightEncoder = rightEncoder;
                prevPerpendicularEncoder = perpendicularEncoder;
                heading = imuHeading;
                initialized = true;
                return;
            }

            // Convert encoder ticks to inches
            double deltaLeft = (leftEncoder - prevLeftEncoder) / ENCODER_TICKS_PER_INCH;
            double deltaRight = (rightEncoder - prevRightEncoder) / ENCODER_TICKS_PER_INCH;
            double deltaPerp = (perpendicularEncoder - prevPerpendicularEncoder) / ENCODER_TICKS_PER_INCH;

            double headingChange = normalizeAngle(imuHeading - heading);

            double deltaX, deltaY;

            // Use hybrid approach based on heading change magnitude
            if (Math.abs(headingChange) < imuAccuracyThreshold) {
                // Line-based odometry for small movements
                deltaX = (deltaLeft + deltaRight) / 2.0;
                deltaY = deltaPerp;
            } else {
                // Arc-based odometry for larger movements
                if (Math.abs(headingChange) < 1e-10) {
                    deltaX = (deltaLeft + deltaRight) / 2.0;
                    deltaY = deltaPerp;
                } else {
                    double radius = (deltaLeft + deltaRight) / (2.0 * headingChange);
                    double strafe = deltaPerp;

                    // Arc calculation
                    deltaX = radius * Math.sin(headingChange) - strafe * Math.sin(headingChange / 2.0);
                    deltaY = radius * (1 - Math.cos(headingChange)) + strafe * Math.cos(headingChange / 2.0);
                }
            }

            // Transform to global coordinates using average heading
            double avgHeading = heading + headingChange / 2.0;
            double cosHeading = Math.cos(avgHeading);
            double sinHeading = Math.sin(avgHeading);

            position.x += deltaX * cosHeading - deltaY * sinHeading;
            position.y += deltaX * sinHeading + deltaY * cosHeading;
            heading = imuHeading;

            // Update previous values
            prevLeftEncoder = leftEncoder;
            prevRightEncoder = rightEncoder;
            prevPerpendicularEncoder = perpendicularEncoder;
        }

        private double normalizeAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        public Point getPosition() { return position.copy(); }
        public double getHeading() { return heading; }
        public void setPosition(Point pos) { position = pos.copy(); }
        public void setHeading(double h) { heading = h; }
    }


