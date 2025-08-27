package org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms;

public class PIDController {
        private double kp, ki, kd, maxIntegral, maxOutput;
        private double integral, previousError;
        private long lastTime;
        private boolean firstRun;

        public PIDController(double kp, double ki, double kd, double maxIntegral, double maxOutput) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.maxIntegral = maxIntegral;
            this.maxOutput = maxOutput;
            this.integral = 0;
            this.previousError = 0;
            this.firstRun = true;
        }

        public PIDController(double kp, double ki, double kd, double maxIntegral) {
            this(kp, ki, kd, maxIntegral, Double.MAX_VALUE);
        }

        public double calculate(double error, double dt) {
            if (dt <= 0) return 0;

            // Anti-windup: only integrate if we're not saturated
            if (Math.abs(integral * ki) < maxOutput) {
                integral += error * dt;
                integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
            }

            double derivative = 0;
            if (!firstRun && dt > 0) {
                derivative = (error - previousError) / dt;
            }

            double output = kp * error + ki * integral + kd * derivative;
            output = Math.max(-maxOutput, Math.min(maxOutput, output));

            previousError = error;
            firstRun = false;

            return output;
        }

        public void reset() {
            integral = 0;
            previousError = 0;
            firstRun = true;
        }

        public void setGains(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }
    }

