package org.firstinspires.ftc.teamcode.team.control;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;

public class StackTracker {
    private int CONES_TAKEN = 0;
    private static final int MAX_CONES_TAKEN = 4;
    private static final double CONE_HEIGHT_DIFFERENCE = 1.34375; //used to be 1.3125, changed 12/21
    private static final double INITIAL_HEIGHT = 9.0d; //used to be 10.25, changed 12/20
    private static final double FEEDER_DUMPER_HEIGHT = 3.625d; //used to be 3.875, changed 12/21

    private static final double INITIAL_POLE_HEIGHT = 14d; //0, 14, 24, 25 (will become 34)
    private static final double POLE_HEIGHT_DIFFERENCE = 10d;
    private static final double MAX_POLE_TARGET_HEIGHT = 25d;
    private int POLE_TARGET_TYPE = 0; //0, 1, 2, 3

    public StackTracker() {
        resetStack();
        resetPoleTargetType();
    }

    //CONES

    public double getExtensionHeight() {
        return getInitialHeight() - getFeederDumperHeight() - (getConesTaken() - 0) * getConeHeightDifference();
    }

    public static double getExtensionHeight(int conesTaken) {
        return  getInitialHeight() - getFeederDumperHeight() - conesTaken * getConeHeightDifference();
    }

    public ResidualVibrationReductionMotionProfilerGenerator motionProfilerSetpoints(boolean extend) {
        if(getConesTaken() == MAX_CONES_TAKEN || getConesTaken() > ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles().length) { //if anything wrong, might be due to this line
            return null;
        }

        return ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles()[getConesTaken() - 0 + (extend ? 0 : getMaxConesTaken())];
    }

    public static ResidualVibrationReductionMotionProfilerGenerator motionProfilerSetpoints(boolean extend, int stonesStacked) {
        if(stonesStacked == 0 || stonesStacked > ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles().length) {
            return null;
        }

        return ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles()[stonesStacked - 1 + (extend ? 0 : getMaxConesTaken())];
    }


    public void resetStack() {
        setConesTaken(0);
    }

    public void takeConeFromStack() {
        if(getConesTaken() >= MAX_CONES_TAKEN) {
            return;
        }

        setConesTaken(getConesTaken() + 1);
    }

    public void addConeToStack() {
        if(getConesTaken() <= 0) {
            return;
        }

        setConesTaken(getConesTaken() - 1);
    }

    @Override
    public String toString() {
        return "Cones taken: " + getConesTaken();
    }

    public static double getConeHeightDifference() {
        return CONE_HEIGHT_DIFFERENCE;
    }

    public static double getInitialHeight() {
        return INITIAL_HEIGHT;
    }

    public static double getFeederDumperHeight() {
        return FEEDER_DUMPER_HEIGHT;
    }

    public int getConesTaken() {
        return CONES_TAKEN;
    }

    public void setConesTaken(int conesTaken) {
        this.CONES_TAKEN = conesTaken;
    }

    public static int getMaxConesTaken(){
        return MAX_CONES_TAKEN;
    }

    //POLES

    public double getPoleExtensionHeight(){
        double returnValue = 0d;
        if (getPoleTargetType() > 0){
            returnValue = getInitialPoleHeight() + (getPoleTargetType() - 1) * getPoleHeightDifference();
            returnValue = Range.clip(returnValue, 0d, getMaxPoleTargetHeight());
        }
        return returnValue;
    }

    public static double getPoleExtensionHeight(int poleType){
        double returnValue = 0d;
        if (poleType > 0){
            returnValue = getInitialPoleHeight() + (poleType - 1) * getPoleHeightDifference();
            returnValue = Range.clip(returnValue, 0d, getMaxPoleTargetHeight());
        }
        return returnValue;
    }

    public ResidualVibrationReductionMotionProfilerGenerator pole_motionProfilerSetpoints(boolean extend){
        if (getPoleTargetType() > 3) { //TODO: Add second check?
            return null;
        }

        return ResidualVibrationReductionMotionProfilerGenerator.getStandardPoleMotionProfiles()[getPoleTargetType() + (extend ? 0 : 4)];
    }

    public ResidualVibrationReductionMotionProfilerGenerator pole_motionProfilerSetpoints(boolean extend, int poleType){
        if (poleType > 3) { //TODO: Add second check?
            return null;
        }

        return ResidualVibrationReductionMotionProfilerGenerator.getStandardPoleMotionProfiles()[poleType + (extend ? 0 : 4)];
    }

    public void resetPoleTargetType(){
        setPoleTargetType(0);
    }

    public void incrementPoleTargetType() {
        if (getPoleTargetType() >= 3){
            return;
        }

        setPoleTargetType(getPoleTargetType() + 1);
    }

    public void decrementPoleTargetType() {
        if (getPoleTargetType() <= 0d){
            return;
        }

        setPoleTargetType(getPoleTargetType() - 1);
    }

    public String toString_PoleType(){
        return "Pole Target Type: " + getPoleTargetType();
    }

    public static double getPoleHeightDifference(){
        return POLE_HEIGHT_DIFFERENCE;
    }

    public static double getInitialPoleHeight(){
        return INITIAL_POLE_HEIGHT;
    }

    public int getPoleTargetType() {
        return POLE_TARGET_TYPE;
    }

    public void setPoleTargetType(int type) {
        this.POLE_TARGET_TYPE = type;
    }

    public static double getMaxPoleTargetHeight(){
        return MAX_POLE_TARGET_HEIGHT;
    }

}
