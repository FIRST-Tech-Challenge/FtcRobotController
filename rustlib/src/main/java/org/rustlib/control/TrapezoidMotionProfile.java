package org.rustlib.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.rustlib.utils.MathHelpers;

import java.util.ArrayList;

public class TrapezoidMotionProfile {
    private final double maxAccel;
    private final double maxVel;
    private final double startPoint;
    private final double endPoint;
    private double resolution;
    private final ArrayList<Setpoint> setpoints;
    private final ElapsedTime timer;

    /**
     * @param maxAccel
     * @param maxVel
     * @param startPoint
     * @param endPoint
     * @param resolution The number of milliseconds in between each point in the generated profile
     */
    public TrapezoidMotionProfile(double maxAccel, double maxVel, double startPoint, double endPoint, double resolution) {
        this.maxAccel = Math.abs(maxAccel);
        this.maxVel = Math.abs(maxVel);
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.resolution = resolution;
        setpoints = generateProfile();
        timer = new ElapsedTime();
    }

    private ArrayList<Setpoint> generateProfile() {
        ArrayList<Setpoint> setpoints = new ArrayList<>();
        int polarity = endPoint > startPoint ? 1 : -1;
        double totalDistance = endPoint - startPoint;
        double accelerationTime = maxVel / maxAccel;
        double accelerationDistance = polarity * 0.5 * maxAccel * Math.pow(accelerationTime, 2);
        double timeAtMaxVel;
        if (2 * Math.abs(accelerationDistance) > Math.abs(totalDistance)) {
            accelerationTime = Math.sqrt(totalDistance / maxAccel);
            accelerationDistance = totalDistance / 2;
            timeAtMaxVel = 0;
        } else {
            timeAtMaxVel = (totalDistance - maxAccel * Math.pow(accelerationTime, 2)) / maxVel;
        }
        double timeUntilDeceleration = accelerationTime + timeAtMaxVel;
        double totalTime = 2 * accelerationTime + timeAtMaxVel;
        for (int i = 0; i < Math.ceil(1000 / resolution * totalTime); i++) {
            double t = i * resolution / 1000;
            if (t <= accelerationTime) {
                setpoints.add(new Setpoint(polarity * 0.5 * maxAccel * Math.pow(t, 2), polarity * maxAccel * t, polarity * maxAccel));
            } else if (t <= accelerationTime + timeAtMaxVel) {
                setpoints.add(new Setpoint(accelerationDistance + maxVel * (t - accelerationTime), polarity * maxVel, 0));
            } else if (t > accelerationTime + timeAtMaxVel && t <= totalTime) {
                setpoints.add(new Setpoint(accelerationDistance + maxVel * timeAtMaxVel, polarity * -maxAccel * (t - timeUntilDeceleration), polarity * -maxAccel));
            } else {
                setpoints.add(new Setpoint(endPoint, 0, 0));
            }
        }
        return setpoints;
    }

    /**
     * @param t The timestamp to sample at, in seconds
     * @return
     */
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
        Setpoint toFollow = sample(t);
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
}
