package org.rustlib.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.rustlib.utils.MathHelpers;

import java.util.ArrayList;

public class MotionProfile {
    private final SystemParams systemParams;
    private final double startPoint;
    private final double endPoint;
    private double resolution = 10; // 1 point for every 10 milliseconds
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
        ArrayList<Setpoint> setpoints = new ArrayList<>();
        int polarity = endPoint > startPoint ? 1 : -1;
        double accelerationTime = systemParams.maxVelocity / systemParams.maxAcceleration;
        double accelerationDistance = polarity * 0.5 * systemParams.maxAcceleration * Math.pow(accelerationTime, 2);
        double timeAtMaxVel = (endPoint - startPoint - systemParams.maxAcceleration * Math.pow(accelerationTime, 2)) / systemParams.maxVelocity;
        double timeUntilDeceleration = accelerationTime + timeAtMaxVel;
        double totalTime = 2 * accelerationTime + timeAtMaxVel;
        for (int i = 0; i < Math.ceil(1000 / resolution * totalTime); i++) {
            double t = i * resolution / 1000;
            if (t <= accelerationTime) {
                setpoints.add(new Setpoint(polarity * 0.5 * systemParams.maxAcceleration * Math.pow(t, 2), polarity * systemParams.maxAcceleration * t, polarity * systemParams.maxAcceleration));
            } else if (t <= accelerationTime + timeAtMaxVel) {
                setpoints.add(new Setpoint(accelerationDistance + systemParams.maxVelocity * (t - accelerationTime), polarity * systemParams.maxVelocity, 0));
            } else if (t > accelerationTime + timeAtMaxVel && t <= totalTime) {
                setpoints.add(new Setpoint(accelerationDistance + systemParams.maxVelocity * timeAtMaxVel, polarity * -systemParams.maxAcceleration * (t - timeUntilDeceleration), polarity * -systemParams.maxAcceleration));
            } else {
                setpoints.add(new Setpoint(endPoint, 0, 0));
            }
        }
        return setpoints;
    }

    public Setpoint sample(double t) {
        double index = 1000 / resolution * t;
        int firstIndex = (int) Math.floor(index);
        int secondIndex = (int) Math.ceil(index);
        Setpoint first = setpoints.get(firstIndex);
        Setpoint second = setpoints.get(secondIndex);
        double tValue = MathHelpers.getTValue(firstIndex, secondIndex, index);
        return new Setpoint(
                MathHelpers.interpolate(tValue, first.position, second.position),
                MathHelpers.interpolate(tValue, first.velocity, second.velocity),
                MathHelpers.interpolate(tValue, first.acceleration, second.acceleration));
    }

    public double calculate(double t) {
        return 0;
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
            this.maxAcceleration = Math.abs(maxAcceleration);
            this.maxVelocity = Math.abs(maxVelocity);
        }
    }
}
