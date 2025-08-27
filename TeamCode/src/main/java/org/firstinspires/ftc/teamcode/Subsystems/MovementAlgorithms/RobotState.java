package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;

public class RobotState {
        public double x, y, heading;
        public double velocityX, velocityY, angularVelocity;
        public long timestamp; // For velocity calculations

        public RobotState(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.velocityX = 0;
            this.velocityY = 0;
            this.angularVelocity = 0;
            this.timestamp = System.currentTimeMillis();
        }

        public Point getPosition() {
            return new Point(x, y);
        }

        public double getSpeed() {
            return Math.sqrt(velocityX * velocityX + velocityY * velocityY);
        }
    }

