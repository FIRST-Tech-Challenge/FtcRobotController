package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;

public class AutoStep implements Cloneable {
    public static final String NO_ACTION = "";
    private int waitMS = 0;
    private int targetX;
    private int targetY;
    private double topSpeed = 0.5;
    private MoveStrategy moveStrategy = MoveStrategy.Curve;
    private RobotDirection robotDirection = RobotDirection.Optimal;
    private String action = NO_ACTION;
    private double desiredHead = BotMoveProfile.DEFAULT_HEADING;
    private boolean continuous = false;
    private String targetReference = "";
    private String conditionValue = "";
    private String conditionFunction = "";

    public int getWaitMS() {
        return waitMS;
    }

    public void setWaitMS(int waitMS) {
        this.waitMS = waitMS;
    }

    public int getTargetX() {
        return targetX;
    }

    public void setTargetX(int targetX) {
        this.targetX = targetX;
    }

    public int getTargetY() {
        return targetY;
    }

    public void setTargetY(int targetY) {
        this.targetY = targetY;
    }

    public double getTopSpeed() {
        return topSpeed;
    }

    public void setTopSpeed(double topSpeed) {
        this.topSpeed = topSpeed;
    }

    public MoveStrategy getMoveStrategy() {
        return moveStrategy;
    }

    public void setMoveStrategy(MoveStrategy moveStrategy) {
        this.moveStrategy = moveStrategy;
    }

    public String getAction() {
        return action;
    }

    public void setAction(String action) {
        this.action = action;
    }

    @Override
    public String toString() {
        return "AutoStep{" +
                "targetX=" + targetX +
                ", targetY=" + targetY +
                ", moveStrategy=" + moveStrategy +
                ", action='" + action + '\'' +
                ", desiredHead=" + desiredHead +
                '}';
    }

    public String getDestination(){
        if (targetReference.equals("")) {
            return String.format("%d : %d", getTargetX(), getTargetY());
        }
        else{
            return targetReference;
        }
    }

    public String getTopSpeedString(){
        return String.format("%.2f", getTopSpeed());
    }

    public String getMoveStrategyString(){
        return getMoveStrategy().name();
    }

    public String getWaitString(){
        return String.format("%d", getWaitMS());
    }

    public double getDesiredHead() {
        return desiredHead;
    }

    public String getDesiredHeadString() {
        return String.format("%.2f", desiredHead);
    }

    public void setDesiredHead(double desiredHead) {
        this.desiredHead = desiredHead;
    }

    public AutoStep clone(){
        AutoStep clone = new AutoStep();
        clone.setWaitMS(this.getWaitMS());
        clone.setTargetX(this.getTargetX());
        clone.setTargetY(this.getTargetY());
        clone.setTopSpeed(this.getTopSpeed());
        clone.setMoveStrategy(this.getMoveStrategy());
        clone.setRobotDirection(this.getRobotDirection());
        clone.setAction(this.getAction());
        clone.setDesiredHead(this.getDesiredHead());
        clone.setContinuous(this.isContinuous());
        clone.setTargetReference(this.getTargetReference());
        clone.setConditionFunction(this.getConditionFunction());
        clone.setConditionValue(this.getConditionValue());
        return clone;
    }

    public boolean isContinuous() {
        return continuous;
    }

    public String isContinuousAsString() {
        return String.format("%b", continuous);
    }


    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public String getTargetReference() {
        return targetReference;
    }

    public void setTargetReference(String targetReference) {
        this.targetReference = targetReference;
    }

    public RobotDirection getRobotDirection() {
        return robotDirection;
    }

    public String getRobotDirectionString() {
        return getRobotDirection().name();
    }

    public void setRobotDirection(RobotDirection robotDirection) {
        this.robotDirection = robotDirection;
    }

    public String getConditionValue() {
        return conditionValue;
    }

    public void setConditionValue(String conditionValue) {
        this.conditionValue = conditionValue;
    }

    public String getConditionFunction() {
        return conditionFunction;
    }

    public void setConditionFunction(String conditionFunction) {
        this.conditionFunction = conditionFunction;
    }
}
