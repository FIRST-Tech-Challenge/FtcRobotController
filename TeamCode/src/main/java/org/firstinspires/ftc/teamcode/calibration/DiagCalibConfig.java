package org.firstinspires.ftc.teamcode.calibration;

import java.util.HashMap;
import java.util.Map;

public class DiagCalibConfig {

    private HashMap<Double,Double> speedDegree = new HashMap<>();
    private double speedPerDegree = 0;

    private MotorReductionBot motorReductions = new MotorReductionBot();

    private double slowDown = 0;

    public DiagCalibConfig(){

    }

    public void setSpeedDegreeData(double speed, double angle){
        speedDegree.put(speed, angle);
    }

    public double getMaxAgle(){
        double maxAgle = 0;
        if (speedDegree.containsKey(0)){
            maxAgle = speedDegree.get(0);
        }
        return maxAgle;
    }

    public double computeSpeedPerDegree(){
        double lowSpeed = 1;
        double highSpeed = 0;
        double agleHigh = 0, angleLow = 0;
        for (Map.Entry<Double, Double> entry : speedDegree.entrySet()) {
            double speed = entry.getKey();
            if (speed == 0){
                continue;
            }
            if (speed <= lowSpeed){
                lowSpeed = speed;
                angleLow = entry.getValue();
            }
            if (speed >= highSpeed){
                highSpeed = speed;
                agleHigh = entry.getValue();
            }
        }

        double speedDiff = highSpeed - lowSpeed;
        double angleDiff = agleHigh - angleLow;

        this.speedPerDegree = speedDiff/angleDiff;

        return getSpeedPerDegree();
    }


    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<Double, Double> entry : speedDegree.entrySet()) {
            sb.append(String.format("%.2f: %.2f ; ", entry.getKey(), entry.getValue()));
        }
        return sb.toString();
    }

    public double getSpeedPerDegree() {
        return speedPerDegree;
    }
}
