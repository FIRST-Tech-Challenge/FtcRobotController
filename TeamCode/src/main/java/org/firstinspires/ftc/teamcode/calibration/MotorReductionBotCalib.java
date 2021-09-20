package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.RobotMovementStats;
import org.firstinspires.ftc.teamcode.bots.RobotVeer;

import java.io.Serializable;


public class MotorReductionBotCalib extends MotorReductionBot implements Serializable {

    private double headChangeBaseline = 0;
    private double MRBaseline = 1;
    private RobotVeer veer = RobotVeer.NONE;
    private RobotDirection direction = RobotDirection.Forward;

    private RobotMovementStats stats = new RobotMovementStats();

    private double LFHeadChange = 0;
    private double LBHeadChange = 0;
    private double RFHeadChange = 0;
    private double RBHeadChange = 0;

    private double originalHeadChange = 0;

    private double leftOdoDistance;
    private double leftOdoDistanceActual;

    private double rightOdoDistance;
    private double rightOdoDistanceActual;

    private double horOdoDistance;
    private double horOdoDistanceActual;

    private double distanceFromTarget;

    public MotorReductionBotCalib(){

    }

    public MotorReductionBotCalib(MotorReductionBot mrb){
        CopyMR(mrb);
    }

    public void CopyMR(MotorReductionBot mrb){
        if (mrb != null) {
            for (int x = 0; x < mrb.motors.length; x++) {
                this.MRs[x] = mrb.MRs[x];
            }

            for (int x = 0; x < mrb.breakSamples.length; x++) {
                this.breakSamples[x] = mrb.breakSamples[x];
            }
        }
    }


    public double getLFHeadChange() {
        return LFHeadChange;
    }


    public void setLFHeadChange(double LFHeadChange) {
        this.LFHeadChange = LFHeadChange;
    }

    public double getLBHeadChange() {
        return LBHeadChange;
    }

    public void setLBHeadChange(double LBHeadChange) {
        this.LBHeadChange = LBHeadChange;
    }

    public double getRFHeadChange() {
        return RFHeadChange;
    }

    public void setRFHeadChange(double RFHeadChange) {
        this.RFHeadChange = RFHeadChange;
    }

    public double getRBHeadChange() {
        return RBHeadChange;
    }

    public void setRBHeadChange(double RBHeadChange) {
        this.RBHeadChange = RBHeadChange;
    }


    public double getHeadChangeBaseline() {
        return headChangeBaseline;
    }

    public void setHeadChangeBaseline(double headChangeOriginal) {
        this.headChangeBaseline = headChangeOriginal;
    }

    public RobotVeer getVeer() {
        return veer;
    }

    public void setVeer(RobotVeer veer) {
        this.veer = veer;
    }

    public RobotDirection getDirection() {
        return direction;
    }

    public void setDirection(RobotDirection direction) {
        this.direction = direction;
    }

    public double getMRBaseline() {
        return MRBaseline;
    }

    public void setMRBaseline(double MRBaseline) {
        this.MRBaseline = MRBaseline;
    }

    public MotorReductionBot getMR(){
        MotorReductionBot mrb = new MotorReductionBot();
        for(int x = 0; x < motors.length; x++){
            mrb.MRs[x] = this.MRs[x];
        }
        mrb.setHeadChange(this.getHeadChange());
        mrb.setDistanceRatio(this.getDistanceRatio());
        return mrb;
    }

    public double getOriginalHeadChange() {
        return originalHeadChange;
    }

    public void setOriginalHeadChange(double originalHeadChange) {
        this.originalHeadChange = originalHeadChange;
    }

    public double getLeftOdoDistance() {
        return leftOdoDistance;
    }

    public void setLeftOdoDistance(double leftOdoDistance) {
        this.leftOdoDistance = leftOdoDistance;
    }

    public double getLeftOdoDistanceActual() {
        return leftOdoDistanceActual;
    }

    public void setLeftOdoDistanceActual(double leftOdoDistanceActual) {
        this.leftOdoDistanceActual = leftOdoDistanceActual;
    }

    public double getRightOdoDistance() {
        return rightOdoDistance;
    }

    public void setRightOdoDistance(double rightOdoDistance) {
        this.rightOdoDistance = rightOdoDistance;
    }

    public double getRightOdoDistanceActual() {
        return rightOdoDistanceActual;
    }

    public void setRightOdoDistanceActual(double rightOdoDistanceActual) {
        this.rightOdoDistanceActual = rightOdoDistanceActual;
    }

    public double getHorOdoDistance() {
        return horOdoDistance;
    }

    public void setHorOdoDistance(double horOdoDistance) {
        this.horOdoDistance = horOdoDistance;
    }

    public double getHorOdoDistanceActual() {
        return horOdoDistanceActual;
    }

    public void setHorOdoDistanceActual(double horOdoDistanceActual) {
        this.horOdoDistanceActual = horOdoDistanceActual;
    }

    public void process(boolean side){
        if (side){
            double linearMovement = Math.abs(leftOdoDistanceActual);
            this.setVeer(RobotVeer.RIGHT);
            if (linearMovement < Math.abs(rightOdoDistanceActual)){
                linearMovement = Math.abs(rightOdoDistanceActual);
                this.setVeer(RobotVeer.LEFT);
            }
            //make it negative for comparison: the larger the value, the better. 0 is the best
            setDistanceRatio(-linearMovement);
        }
        else {
            if (Math.abs(leftOdoDistanceActual) > Math.abs(rightOdoDistanceActual)) {
                double left = Math.abs(rightOdoDistanceActual / leftOdoDistanceActual);
                setDistanceRatio(left);
                this.setVeer(RobotVeer.RIGHT);
            }
            if (Math.abs(rightOdoDistanceActual) > Math.abs(leftOdoDistanceActual)) {
                double right = Math.abs(leftOdoDistanceActual / rightOdoDistanceActual);
                setDistanceRatio(right);
                this.setVeer(RobotVeer.LEFT);
            }
        }
    }


    public String getSelectedIndicator(int index){
        if (this.selectedIndex == index){
            return "*";
        }
        return " ";
    }

    public double getOverDriveLeft(){
        double diff = 0;
        double desired = Math.abs(this.getLeftOdoDistance());
        double actual = Math.abs(this.getLeftOdoDistanceActual());
        diff = actual - desired;
        //negative if shorter than desired
        return diff;
    }

    public double getOverDriveRight(){
        double diff = 0;
        double desired = Math.abs(this.getRightOdoDistance());
        double actual = Math.abs(this.getRightOdoDistanceActual());
        diff = actual - desired;
        //negative if shorter than desired
        return diff;
    }

    public RobotMovementStats getStats() {
        return stats;
    }

    public void setStats(RobotMovementStats stats) {
        this.stats = stats;
    }

    public double getDistanceFromTarget() {
        return distanceFromTarget;
    }

    public void setDistanceFromTarget(double distanceFromTarget) {
        this.distanceFromTarget = distanceFromTarget;
    }
}
