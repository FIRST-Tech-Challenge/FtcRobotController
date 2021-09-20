package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;

import java.util.ArrayList;

public class RobotMovementStats {
    private double totalDistance;
    private double totalTime;
    private double averageSpeed;
    private double accelerateTime;
    private double accelerateDistance;
    private double fullSpeedTime;
    private double fullSpeedDistance;
    private double fullSpeed;
    private double slowDownTime;
    private double slowDownDistance;
    private double motorPower;
    private double slowDownDelay = 0;
    private MotorReductionBot suggestedMR = new MotorReductionBot();

    private double maxVelocityLeft = 0;
    private double maxVelocityRight = 0;

    ArrayList<MotorReductionBot> motorAmps = new ArrayList<>();

    ElapsedTime fullSpeedTimer = new ElapsedTime();
    double startingPoint = 0;
    double startFullSpeedPoint = 0;
    double slowDownPoint = 0;

    ElapsedTime accelerateTimer = new ElapsedTime();
    ElapsedTime slowDownTimer = new ElapsedTime();

    public double getTotalDistance() {
        return totalDistance;
    }

    public void setTotalDistance(double totalDistance) {
        this.totalDistance = totalDistance;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public void setTotalTime(double totalTime) {
        this.totalTime = totalTime;
    }

    public double getAverageSpeed() {
        return averageSpeed;
    }

    public void setAverageSpeed(double averageSpeed) {
        this.averageSpeed = averageSpeed;
    }

    public double getAccelerateTime() {
        return accelerateTime;
    }

    public void setAccelerateTime(double accelerateTime) {
        this.accelerateTime = accelerateTime;
    }

    public double getAccelerateDistance() {
        return accelerateDistance;
    }

    public void setAccelerateDistance(double accelerateDistance) {
        this.accelerateDistance = accelerateDistance;
    }

    public double getFullSpeedTime() {
        return fullSpeedTime;
    }

    public void setFullSpeedTime(double fullSpeedTime) {
        this.fullSpeedTime = fullSpeedTime;
    }

    public double getFullSpeedDistance() {
        return fullSpeedDistance;
    }

    public void setFullSpeedDistance(double fullSpeedDistance) {
        this.fullSpeedDistance = fullSpeedDistance;
    }

    public double getFullSpeed() {
        return fullSpeed;
    }

    public void setFullSpeed(double fullSpeed) {
        this.fullSpeed = fullSpeed;
    }

    public double getSlowDownTime() {
        return slowDownTime;
    }

    public void setSlowDownTime(double slowDownTime) {
        this.slowDownTime = slowDownTime;
    }

    public double getSlowDownDistance() {
        return slowDownDistance;
    }

    public double getSlowDownDistanceRaw() {
        return slowDownDistance*YellowBot.COUNTS_PER_INCH_REV;
    }

    public void setSlowDownDistance(double slowDownDistance) {
        this.slowDownDistance = slowDownDistance;
    }

    public double getSlowDownDelta(){
        return this.getFullSpeed()/2 * getFullSpeedTime()/1000;
    }

    //accelerate
    public void startAccelerateTimer(double startingPoint){
        this.startingPoint = startingPoint;
        accelerateTimer.reset();
    }

    public void stopAccelerateTimer(double stopAcceleratePoint){
        setAccelerateTime(accelerateTimer.milliseconds());
        this.setAccelerateDistance(Math.abs(stopAcceleratePoint - startingPoint)/YellowBot.COUNTS_PER_INCH_REV);
    }

    //full speed
    public void startFullSpeedTimer(double startingPoint){
        this.startFullSpeedPoint = startingPoint;
        fullSpeedTimer.reset();
    }

    public void stopFullSpeedTimer(double stopPoint){
        setFullSpeedTime(fullSpeedTimer.milliseconds());
        this.setFullSpeedDistance(Math.abs(stopPoint - startFullSpeedPoint)/YellowBot.COUNTS_PER_INCH_REV);
        this.setFullSpeed(fullSpeedDistance/fullSpeedTime*1000);
    }

    //slow down
    public void startSlowDownTimer(double slowdownPointActual, double slowdownPointDesired){
        this.slowDownPoint = slowdownPointActual;
        if (Math.abs(slowdownPointDesired) < Math.abs(slowdownPointActual)){
            this.slowDownDelay =  Math.abs(slowdownPointActual) - Math.abs(slowdownPointDesired);
        }
        slowDownTimer.reset();
    }

    public void stopSlowdownTimer(double stopPoint){
        setSlowDownTime(slowDownTimer.milliseconds());
        this.setSlowDownDistance((Math.abs(stopPoint - slowDownPoint) + this.slowDownDelay)/YellowBot.COUNTS_PER_INCH_REV);
    }

    //totals
    public void computeTotals(double stopPoint){
        this.setTotalDistance(Math.abs(stopPoint - startingPoint)/YellowBot.COUNTS_PER_INCH_REV);
        this.setTotalTime(this.getAccelerateTime() + this.getFullSpeedTime() + this.getSlowDownTime());
        this.setAverageSpeed(this.getTotalDistance()/this.getTotalTime()*1000);
    }

    public double getMotorPower() {
        return motorPower;
    }

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public void addAmpSample(MotorReductionBot sample){
        this.motorAmps.add(sample);
    }

    public int getAmpSampleCount(){
        return this.motorAmps.size();
    }

    public MotorReductionBot getMotorAmpsAverages(){
        MotorReductionBot average = new MotorReductionBot();
        for (MotorReductionBot sample : this.motorAmps) {
            average.setRB(average.getRB() + sample.getRB());
            average.setRF(average.getRF() + sample.getRF());
            average.setLB(average.getLB() + sample.getLB());
            average.setLF(average.getLF() + sample.getLF());
        }

        average.setRB(average.getRB()/this.motorAmps.size());
        average.setRF(average.getRF()/this.motorAmps.size());
        average.setLB(average.getLB()/this.motorAmps.size());
        average.setLF(average.getLF()/this.motorAmps.size());
        return average;
    }

    public MotorReductionBot getSuggestedMR() {
        return suggestedMR;
    }

    public void setSuggestedMR(MotorReductionBot suggestedMR) {
        this.suggestedMR = suggestedMR;
    }

    public void updateVelocity(double left, double right){
        if (getMaxVelocityLeft() < left){
            maxVelocityLeft = left;
        }

        if (getMaxVelocityRight() < right){
            maxVelocityRight = right;
        }
    }

    public double getMaxVelocityLeft() {
        return maxVelocityLeft;
    }

    public double getMaxVelocityRight() {
        return maxVelocityRight;
    }
}
