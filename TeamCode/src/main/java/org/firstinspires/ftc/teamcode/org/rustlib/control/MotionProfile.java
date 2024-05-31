package org.firstinspires.ftc.teamcode.org.rustlib.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class MotionProfile {
    private final SystemParams systemParams;
    private final double startPoint;
    private final double endPoint;
    private final ArrayList<Setpoint> setpoints;
    private final ElapsedTime timer;

    public MotionProfile(SystemParams systemParams, double startPoint, double endPoint) {
        this.systemParams = systemParams;
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        setpoints = generateProfile();
        timer = new ElapsedTime();
    }

    private ArrayList<Setpoint> generateProfile() {
        return null;
    }

    public Setpoint sample(double t) {
        return null;
    }

    public static class Setpoint {
        public final double position;
        public final double velocity;
        public final double acceleration;

        public Setpoint(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    public static class SystemParams {
        public final double maxAcceleration;
        public final double maxVelocity;

        public SystemParams(double maxAcceleration, double maxVelocity) {
            this.maxAcceleration = maxAcceleration;
            this.maxVelocity = maxVelocity;
        }
    }
}
