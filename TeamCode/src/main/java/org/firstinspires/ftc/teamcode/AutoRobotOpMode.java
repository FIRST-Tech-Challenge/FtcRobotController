package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
public final class AutoRobotOpMode {

    public static enum RobotMovement {
        FORWARD(1, 0, 0),
        BACKWARD(-1, 0, 0),
        LEFT(0, -1, 0),
        RIGHT(0, 1, 0),
        FORWARD1(1, 0, 0, 1d),
        BACKWARD1(-1, 0, 0, 1d),
        LEFT1(0, -1, 0, 1d),
        RIGHT1(0, 1, 0, 1d),
        TURN_LEFT90(-90f),
        TURN_RIGHT90(90f),
        TURN_LEFT45(-45f),
        TURN_RIGHT45(45f),
        TURN_LEFT180(-180f),
        TURN_RIGHT180(180f);

        final double TIME;
        /**
         * FORWARD AND BACKWARD
         */
        final double AXIAL;
        /**
         * STRAFING, SIDE TO SIDE
         */
        final double LATERAL;
        /**
         * ROTATION
         */
        final double YAW;

        public double getAngle() {
            return YAW;
        }

        RobotMovement() {
            this.YAW = 0;
            this.TIME = -1;
            this.AXIAL = 0;
            this.LATERAL = 0;
        }

        RobotMovement(double yaw) {
            this.YAW = yaw;
            this.TIME = -1;
            this.AXIAL = 0;
            this.LATERAL = 0;
        }

        RobotMovement(double yaw, double time) {
            this.YAW = yaw;
            this.TIME = time;
            this.AXIAL = 1;
            this.LATERAL = 0;
        }

        RobotMovement (double axial, double lateral, double yaw) {
            this.YAW = yaw;
            this.TIME = -1;
            this.AXIAL = axial;
            this.LATERAL = lateral;
        }

        RobotMovement (double axial, double lateral, double yaw, double time) {
            this.YAW = yaw;
            this.TIME = time;
            this.AXIAL = axial;
            this.LATERAL = lateral;
        }

        public double getTime() {
            return TIME;
        }

        public double getAxial() {
            return AXIAL;
        }

        public double getLateral() {
            return LATERAL;
        }

        public double getYaw() {
            return YAW;
        }

        public double[] getMovement() {
            return new double[] {AXIAL, LATERAL, YAW, TIME};
        }
    }

    public static RobotMovement[] movements;

    @Autonomous(name = "AutoRobotOpMode")
    public static class AutonomousRobotOpMode extends RobotOpMode {
        @Override
        public void gamePadMoveRobot() {
            // Do nothing
        }

        @Override
        public void init() {
            super.init();
            // Do nothing
        }

        @Override
        public void robotLoop() {

        }
    }
}
