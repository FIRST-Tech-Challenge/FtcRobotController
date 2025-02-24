package org.firstinspires.ftc.teamcode.util.drivetrain;

import com.arcrobotics.ftclib.geometry.Vector2d;

public class MecanumChassisUtils {
    /**
     * @param vector - hSpeed, vSpeed
     * @param rSpeed - rotationSpeed
     * @return MecanumWheelSpeeds
     */
    public static MecanumWheelSpeeds chassisSpeedToWheelSpeeds(Vector2d vector, double rSpeed) {
        double[] speeds = new double[4];
        speeds[0] = Math.sin(vector.angle() + Math.PI / 4) + rSpeed; // front left
        speeds[1] = Math.sin(vector.angle() - Math.PI / 4) - rSpeed; // front right
        speeds[2] = Math.sin(vector.angle() - Math.PI / 4) + rSpeed; // back left
        speeds[3] = Math.sin(vector.angle() + Math.PI / 4) - rSpeed; // back right

        normalize(speeds, vector.magnitude());

        speeds[0] += rSpeed;
        speeds[1] -= rSpeed;
        speeds[2] += rSpeed;
        speeds[3] -= rSpeed;

        normalize(speeds);

        return new MecanumWheelSpeeds(speeds);
    }

    private static void normalize(double[] wheelSpeeds, double mag) {
        double maxSpeed = getMaxSpeeds(wheelSpeeds);

        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxSpeed) * mag;
        }
    }

    private static void normalize(double[] wheelSpeeds) {
        double maxSpeed = getMaxSpeeds(wheelSpeeds);

        if(maxSpeed > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxSpeed;
            }
        }
    }

    private static double getMaxSpeeds(double[] wheelSpeeds) {
        double maxSpeed = 0;
        for(double speed : wheelSpeeds) {
            maxSpeed = Math.max(maxSpeed, Math.abs(speed));
        }
        return maxSpeed;
    }

    public static class MecanumWheelSpeeds {
        private final double[] speeds;

        public MecanumWheelSpeeds(double[] speeds) {
            this.speeds = speeds;
        }

        public double getFrontLeft() {
            return speeds[0];
        }

        public double getFrontRight() {
            return speeds[1];
        }

        public double getBackLeft() {
            return speeds[2];
        }

        public double getBackRight() {
            return speeds[3];
        }

        public void setFrontLeft(double speed) {
            speeds[0] = speed;
        }

        public void setFrontRight(double speed) {
            speeds[1] = speed;
        }

        public void setBackLeft(double speed) {
            speeds[2] = speed;
        }

        public void setBackRight(double speed) {
            speeds[3] = speed;
        }

        public double[] getSpeeds() {
            return speeds;
        }

        public MecanumWheelSpeeds mul(double x) {
            double[] newSpeeds = new double[4];
            for (int i = 0; i < this.speeds.length; i++) {
                newSpeeds[i] = this.speeds[i] * x;
            }

            return new MecanumWheelSpeeds(newSpeeds);
        }
    }
}
