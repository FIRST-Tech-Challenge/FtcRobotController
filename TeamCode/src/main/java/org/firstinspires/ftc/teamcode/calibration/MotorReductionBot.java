package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.YellowBot;

import java.io.Serializable;
import java.math.BigDecimal;


public class MotorReductionBot implements Serializable {

    public static double DEFAULT_REDUCTION = 1;
    protected int selectedIndex = 0;
    private double distanceRatio = 1;
    private double headChange = 0;


    protected MotorName [] motors = new MotorName[]{MotorName.LF, MotorName.RF, MotorName.RB, MotorName.LB};
    protected double [] MRs = new double[] {DEFAULT_REDUCTION, DEFAULT_REDUCTION, DEFAULT_REDUCTION, DEFAULT_REDUCTION};

    public static double [] POWER_SAMPLES = new double[]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    protected double [] breakSamples = new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0};

    public MotorReductionBot(){

    }

    public double getLF() {
        return MRs[0];
    }

    public void setLF(double LF) {
        MRs[0] = LF;
    }

    public double getRF() {
        return MRs[1];
    }

    public void setRF(double RF) {
        MRs[1] = RF;
    }

    public double getRB() {
        return MRs[2];
    }

    public void setRB(double RB) {
        MRs[2] = RB;
    }

    public double getLB() {
        return MRs[3];
    }

    public void setLB(double LB) {
        MRs[3] = LB;
    }


    public boolean isSelected(int index){
        return this.selectedIndex == index;
    }


    public void inrementSelectedMR(boolean tenIncrement){
        double val = 0.01;
        if (tenIncrement){
            val = val * 10;
        }
        this.MRs[selectedIndex] = this.MRs[selectedIndex] + val;
        if (this.MRs[selectedIndex] > 1){
            this.MRs[selectedIndex] = 1;
        }
    }

    public void decrementSelectedMR(boolean tenIncrement){
        double val = 0.01;
        if (tenIncrement){
            val = val * 10;
        }
        this.MRs[selectedIndex] = this.MRs[selectedIndex] - val;
        if (this.MRs[selectedIndex] < 0){
            this.MRs[selectedIndex] = 0;
        }
    }

    public void selectNext(){
        if (selectedIndex + 1 >= motors.length){
            selectedIndex = 0;
        }
        else{
            selectedIndex++;
        }
    }

    public void selectPrev(){
        if (selectedIndex - 1 < 0){
            selectedIndex = motors.length - 1;
        }
        else{
            selectedIndex--;
        }
    }

    public double getDistanceRatio() {
        return distanceRatio;
    }

    public void setDistanceRatio(double distanceRatio) {
        this.distanceRatio = distanceRatio;
    }

    public double getHeadChange() {
        return headChange;
    }

    public void setHeadChange(double headChange) {
        this.headChange = headChange;
    }

    public boolean compare(MotorReductionBot another){
        if (another == null){
            return true;
        }
        return Math.abs(this.headChange) <= Math.abs(another.getHeadChange());
    }

    public double getBreakPoint(double power) {
        power = Math.round(power*10)/10.0;
        int index = getPowerIndex(power);
        if (index < 0){
            return 0;
        }
        else{
            return this.breakSamples[index];
        }
    }

    public double getBreakPointInches(double power) {
        double raw = getBreakPoint(power);
        return raw/YellowBot.COUNTS_PER_INCH_REV;
    }


    public void setBreakPoint(double breakPoint, double power) {
        int index = getPowerIndex(power);
        this.breakSamples[index] = breakPoint;
    }

    public static int getPowerIndex(double power){
        power = Math.round(power*10)/10.0;
        int v1 = (int)Math.round(power*10);
        int index = -1;
        if (v1 >= 1 && v1 <= POWER_SAMPLES.length){
            index = v1 - 1;
        }
//        int index = -1;
//        for (int i = 0; i < POWER_SAMPLES.length; i++){
//            int v2 = (int)Math.round(POWER_SAMPLES[i]*10);
//            if (Integer.compare(v1, v2) == 0){
//                index = i;
//                break;
//            }
//        }
        return index;
    }

    public void updateBreakSamples(MotorReductionBot input){
        for (int i = 0; i < POWER_SAMPLES.length; i++){
            double power = POWER_SAMPLES[i];
            this.setBreakPoint(input.getBreakPoint(power), power);
        }
    }
}
