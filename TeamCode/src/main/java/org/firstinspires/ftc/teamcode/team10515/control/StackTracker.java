package org.firstinspires.ftc.teamcode.team10515.control;

import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;

public class StackTracker {
    private static final int MAX_STONE_SKYSCRAPER_COUNT = 8;
    private static final double STONE_HEIGHT = 4d;
    private static final double FOUNDATION_HEIHGT = 1d;
    private static final double FEEDER_DUMPER_HEIGHT = 5d;

    private int stonesStacked;

    public StackTracker() {
        resetStack();
    }

    public double getExtensionHeight() {
        return getFoundationHeihgt() - getFeederDumperHeight() + (getStonesStacked() + 1) * getStoneHeight();
    }

    public ResidualVibrationReductionMotionProfilerGenerator motionProfilerSetpoints(boolean extend) {
        if(getStonesStacked() == 0 || getStonesStacked() > ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles().length) {
            return null;
        }

        return ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles()[getStonesStacked() - 1 + (extend ? 0 : getMaxStoneSkyscraperCount())];
    }

    public static ResidualVibrationReductionMotionProfilerGenerator motionProfilerSetpoints(boolean extend, int stonesStacked) {
        if(stonesStacked == 0 || stonesStacked > ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles().length) {
            return null;
        }

        return ResidualVibrationReductionMotionProfilerGenerator.getStandardMotionProfiles()[stonesStacked - 1 + (extend ? 0 : getMaxStoneSkyscraperCount())];
    }

    public static double getExtensionHeight(int stonesStacked) {
        return getFoundationHeihgt() - getFeederDumperHeight() + (stonesStacked + 1) * getStoneHeight();
    }

    public void resetStack() {
        setStonesStacked(0);
    }

    public void addStoneToStack() {
        if(getStonesStacked() >= 20) {
            return;
        }

        setStonesStacked(getStonesStacked() + 1);
    }

    public void removeStoneFromStack() {
        if(getStonesStacked() <= 0) {
            return;
        }

        setStonesStacked(getStonesStacked() - 1);
    }

    @Override
    public String toString() {
        return "Stones stacked: " + getStonesStacked();
    }

    public static double getStoneHeight() {
        return STONE_HEIGHT;
    }

    public static double getFoundationHeihgt() {
        return FOUNDATION_HEIHGT;
    }

    public static double getFeederDumperHeight() {
        return FEEDER_DUMPER_HEIGHT;
    }

    public int getStonesStacked() {
        return stonesStacked;
    }

    public void setStonesStacked(int stonesStacked) {
        this.stonesStacked = stonesStacked;
    }

    public static int getMaxStoneSkyscraperCount() {
        return MAX_STONE_SKYSCRAPER_COUNT;
    }
}
