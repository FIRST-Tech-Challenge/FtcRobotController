package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.trajectory.Functions.AccelQuartic;
import org.firstinspires.ftc.teamcode.trajectory.Functions.DeaccelQuartic;
import org.firstinspires.ftc.teamcode.trajectory.Functions.Horizontal;
import org.firstinspires.ftc.teamcode.trajectory.Functions.ProfileFunction;

import java.util.ArrayList;

public class MotionProfile {

    ProfileFunction[] phases;
    Double[] endTimes;

    private MotionProfile(ProfileFunction[] phases, Double[] endTimes) {
        this.phases = phases;
        this.endTimes = endTimes;
    }

    public double getVelocity(double t) {
        for (int i = 0; i < endTimes.length; i++) {
            if (t < endTimes[i]) {
                if (i == 0) {
                    return phases[i].getVelocity(t);
                } else {
                    return phases[i].getVelocity(t - endTimes[i-1]);
                }
            }
        }
        return 0;
    }

    public double getPosition(double t) {
        for (int i = 0; i < endTimes.length; i++) {
            if (t < endTimes[i]) {
                if (i == 0) {
                    return phases[i].getPosition(t);
                } else {
                    return phases[i].getPosition(t - endTimes[i-1]);
                }
            }
        }
        return 0;
    }

    public double getTotalTime() {
        return endTimes[endTimes.length - 1];
    }


    public static class Builder {

        private ArrayList<ProfileFunction> funcs = new ArrayList<ProfileFunction>();
        private ArrayList<Double> stops = new ArrayList<Double>();
        private Double totalTime = 0d;
        private double currentPos = 0d;

        public Builder() {
            funcs.clear();
            stops.clear();
            totalTime = 0d;
            currentPos = 0d;
        }

        public Builder addAccelPhase(double accelTime, double maxVelo, double start) {
            funcs.add(new AccelQuartic(accelTime, maxVelo, start, currentPos));
            totalTime += accelTime;
            currentPos += funcs.get(funcs.size() - 1).getPosition(accelTime);
            stops.add(totalTime);
            return this;
        }

        public Builder addDeaccelPhase(double accelTime, double maxVelo, double end) {
            funcs.add(new DeaccelQuartic(accelTime, maxVelo, end, currentPos));
            totalTime += accelTime;
            stops.add(totalTime);
            currentPos += funcs.get(funcs.size() - 1).getPosition(accelTime);
            return this;
        }

        public Builder addSteadyPhase(double speed, double time) {
            funcs.add(new Horizontal(speed, currentPos));
            totalTime += time;
            stops.add(time);
            currentPos += funcs.get(funcs.size() - 1).getPosition(time);
            return this;
        }

        public MotionProfile build() {
            return new MotionProfile((ProfileFunction[]) funcs.toArray(), (Double[]) stops.toArray());
        }
    }
}
