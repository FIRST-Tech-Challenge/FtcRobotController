package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;

public class MecanumDriveCalibration {
        private static final double SQRT_2 = Math.sqrt(2);

        private double frontLeftAngle = Math.PI / 4;    // 45 degrees
        private double frontRightAngle = -Math.PI / 4;  // -45 degrees
        private double backLeftAngle = -Math.PI / 4;    // -45 degrees
        private double backRightAngle = Math.PI / 4;    // 45 degrees
        private boolean calibrated;

        public MecanumDriveCalibration() {
            calibrated = false;
        }

        public void calibrate(double forwardVelocity, double sidewaysVelocity) {
            double actualAngle = Math.atan2(sidewaysVelocity, forwardVelocity);

            frontLeftAngle = actualAngle + Math.PI / 4;
            frontRightAngle = actualAngle - Math.PI / 4;
            backLeftAngle = actualAngle - Math.PI / 4;
            backRightAngle = actualAngle + Math.PI / 4;

            calibrated = true;
        }

        public double[] calculateMotorPowers(double driveX, double driveY, double turn, boolean fieldCentric, double robotHeading) {
            // Field-centric transformation
            if (fieldCentric) {
                double temp = driveX * Math.cos(-robotHeading) - driveY * Math.sin(-robotHeading);
                driveY = driveX * Math.sin(-robotHeading) + driveY * Math.cos(-robotHeading);
                driveX = temp;
            }

            // Standard mecanum drive calculations
            double frontLeft = driveX + driveY + turn;
            double frontRight = driveX - driveY - turn;
            double backLeft = driveX - driveY + turn;
            double backRight = driveX + driveY - turn;

            // Normalize to stay within [-1, 1]
            double maxPower = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                    Math.max(Math.abs(backLeft), Math.abs(backRight))
            ));

            return new double[]{
                    frontLeft / maxPower,
                    frontRight / maxPower,
                    backLeft / maxPower,
                    backRight / maxPower
            };
        }

        public double[] calculateMotorPowers(double driveX, double driveY, double turn) {
            return calculateMotorPowers(driveX, driveY, turn, false, 0);
        }
    }
