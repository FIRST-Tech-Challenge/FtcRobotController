package org.firstinspires.ftc.teamcode.calibration;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.Serializable;

public class BotCalibConfig implements Serializable {
    public static String BOT_CALIB_CONFIG = "bot-config.json";
    private double wheelBaseSeparation;
    private double horizontalTicksDegree;
    private double horizontalTicksDegreeLeft;
    private double horizontalTicksDegreeRight;
    private double minRadiusLeft;
    private double minRadiusRight;
    private MotorReductionBot strafeLeftReduction;
    private MotorReductionBot strafeRightReduction;
    private MotorReductionBot diagMRLeft;
    private MotorReductionBot diagMRRight;
    private MotorReductionBot moveMRForward;
    private MotorReductionBot moveMRBack;
    private MotorReductionBot spinLeftConfig = new MotorReductionBot();
    private MotorReductionBot spinRightConfig  = new MotorReductionBot();;


    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static BotCalibConfig deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, BotCalibConfig.class);
    }


    public double getWheelBaseSeparation() {
        return wheelBaseSeparation;
    }

    public void setWheelBaseSeparation(double wheelBaseSeparation) {
        this.wheelBaseSeparation = wheelBaseSeparation;
    }

    public double getHorizontalTicksDegree() {
        return horizontalTicksDegree;
    }

    public void updateHorizontalTicksDegree() {

        this.horizontalTicksDegree = (this.getHorizontalTicksDegreeLeft() + this.getHorizontalTicksDegreeRight())/2;
    }

    public double getMinRadiusLeft() {
        return minRadiusLeft;
    }

    public void setMinRadiusLeft(double minRadiusLeft) {
        this.minRadiusLeft = minRadiusLeft;
    }

    public double getMinRadiusRight() {
        return minRadiusRight;
    }

    public void setMinRadiusRight(double minRadiusRight) {
        this.minRadiusRight = minRadiusRight;
    }

    public MotorReductionBot getStrafeLeftReduction() {
        return strafeLeftReduction;
    }

    public void setStrafeLeftReduction(MotorReductionBot strafeLeftReduction) {
        this.strafeLeftReduction = strafeLeftReduction;
    }

    public MotorReductionBot getStrafeRightReduction() {
        return strafeRightReduction;
    }

    public void setStrafeRightReduction(MotorReductionBot strafeRightReduction) {
        this.strafeRightReduction = strafeRightReduction;
    }

    public double getHorizontalTicksDegreeLeft() {
        return horizontalTicksDegreeLeft;
    }

    public void setHorizontalTicksDegreeLeft(double horizontalTicksDegreeLeft) {
        this.horizontalTicksDegreeLeft = horizontalTicksDegreeLeft;
        updateHorizontalTicksDegree();
    }

    public double getHorizontalTicksDegreeRight() {
        return horizontalTicksDegreeRight;
    }

    public void setHorizontalTicksDegreeRight(double horizontalTicksDegreeRight) {
        this.horizontalTicksDegreeRight = horizontalTicksDegreeRight;
        updateHorizontalTicksDegree();
    }


    public MotorReductionBot getMoveMRForward() {
        return moveMRForward;
    }

    public void setMoveMRForward(MotorReductionBot moveMRForward) {
        this.moveMRForward = moveMRForward;
    }

    public MotorReductionBot getMoveMRBack() {
        return moveMRBack;
    }

    public void setMoveMRBack(MotorReductionBot moveMRBack) {
        this.moveMRBack = moveMRBack;
    }

    public MotorReductionBot getDiagMRLeft() {
        return diagMRLeft;
    }

    public void setDiagMRLeft(MotorReductionBot diagMRLeft) {
        this.diagMRLeft = diagMRLeft;
    }

    public MotorReductionBot getDiagMRRight() {
        return diagMRRight;
    }

    public void setDiagMRRight(MotorReductionBot diagMRRight) {
        this.diagMRRight = diagMRRight;
    }

    public MotorReductionBot getSpinLeftConfig() {
        return spinLeftConfig;
    }

    public void setSpinLeftConfig(MotorReductionBot spinLeftConfig) {
        this.spinLeftConfig = spinLeftConfig;
    }

    public MotorReductionBot getSpinRightConfig() {
        return spinRightConfig;
    }

    public void setSpinRightConfig(MotorReductionBot spinRightConfig) {
        this.spinRightConfig = spinRightConfig;
    }
}
