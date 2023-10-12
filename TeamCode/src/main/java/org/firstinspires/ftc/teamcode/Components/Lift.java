package org.firstinspires.ftc.teamcode.Components;


import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

/**
 * William
 */
public class Lift extends RFDualMotor {
    private double lastPower = 0.0;
    private double target = 0.0;
    private double MIN_VELOCITY = 20, MANUAL_TIME = 0.2, lastManualTime = -1.0;


    /**
     * Constructor
     */
    public Lift() {
        super("leftLiftMotor", "rightLiftMotor",true);
        super.setDirection(DcMotorSimple.Direction.REVERSE);
        super.setConstants(134, 0.0, 3.2786E-4, 7.286E-4, 2440, -2280, 1097, -66974, 0, 0);
    }

    /**
     * Stores different states of lift.
     */
    public enum LiftPositionStates {
        HIGH_SET_LINE(3000, false),
        MID_SET_LINE(2000, false),
        LOW_SET_LINE(1000, false),
        AT_ZERO(0, true);

        double position;
        boolean state;

        LiftPositionStates(double p_position, boolean p_state) {
            this.position = p_position;
            this.state = p_state;
        }

        void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < LiftPositionStates.values().length; i++) {
                    if(LiftPositionStates.values()[i].state){
                        LOGGER.log("assigned false to position state: "+LiftPositionStates.values()[i].name());
                    }
                    LiftPositionStates.values()[i].state = false;

                }
                this.state = true;
                LOGGER.log(RFLogger.Severity.INFO, "assigned true to position state: " + this.name());
            }
        }

        public double getPosition() {
            return position;
        }
    }

    public enum LiftMovingStates {

        HIGH(false),
        MID(false),
        LOW(false),
        AT_ZERO(true);

        boolean state;

        LiftMovingStates(boolean p_state) {
            this.state = p_state;
        }

        void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < LiftMovingStates.values().length; i++) {
                    if(LiftMovingStates.values()[i].state){
                        LOGGER.log("assigned false to target state: "+LiftMovingStates.values()[i].name());
                    }
                    LiftMovingStates.values()[i].state = false;

                }
                this.state = true;
                LOGGER.log(RFLogger.Severity.INFO, "assigned true to target state: " + this.name());
            }
        }
    }

    /**
     * Depending on which state the lift is currently in, checks whether the state can be
     * transitioned to the next state, then changes state values.
     * Logs which state(s)' values have been changed and to what.
     * Logs to RFMotor & general logs.
     * Logs to finest level.
     * Updates LiftMovingStates and LiftPositionStates state machines.
     */
    public void update() {
        LOGGER.log(RFLogger.Severity.FINEST, "currentPos: " + super.getCurrentPosition() +", currentTarget: "+super.getTarget());
        for(var i : LiftPositionStates.values()){
            if(abs(super.getCurrentPosition()-i.position)<20){
                i.setStateTrue();
            }
        }
        for(var i : LiftMovingStates.values()){
            if(i.state&&super.getTarget()!=LiftPositionStates.values()[i.ordinal()].position){
                setPosition(LiftPositionStates.values()[i.ordinal()]);
            }
        }
        if(time-lastManualTime<MANUAL_TIME){
            super.setPosition(super.getTarget(),0);
        }
        else{
            super.setTarget(super.getCurrentPosition());
        }
        LOGGER.log("currentPos: "+super.getCurrentPosition());

    }

    /**
     * Sets target position for lift.
     *
     * @param p_target target position for lift to run to
     *                 Logs what position the target position has been set to.
     *                 Logs to RFMotor & general logs.
     *                 Logs to finest level.
     *                 Updates LiftMovingStates state machine.
     */
    public void setPosition(double p_target) {
        if(!Wrist.WristStates.FLAT.state) {
            super.setPosition(p_target, 0);
            if (target != p_target) {
                LOGGER.setLogLevel(RFLogger.Severity.INFO);
                LOGGER.log("lifting to: " + p_target);
                target = p_target;
            }
        }
        else{
            LOGGER.log(RFLogger.Severity.SEVERE, "Wrist state FLAT, can't move");
        }
    }

    public void setPosition(LiftPositionStates p_state) {
        if(!Wrist.WristStates.FLAT.state) {
            super.setPosition(p_state.position, 0);
//            if (target != p_state.position) {
                LOGGER.setLogLevel(RFLogger.Severity.INFO);
                LOGGER.log("lifting to: " + p_state.position);
                target = p_state.position;
//            }
        }
        else{
            LOGGER.log(RFLogger.Severity.SEVERE, "Wrist state FLAT, can't move");
        }
        if(!LiftMovingStates.values()[p_state.ordinal()].state){
            LiftMovingStates.values()[p_state.ordinal()].setStateTrue();
        }
    }

    /**
     * Manually extend/retract slides
     *
     * @param p_power How fast the user wants to move the slides and in what direction
     *                Logs that the lift is currently being manually extended.
     *                Logs to RFMotor & general logs.
     *                Logs to finest level.
     *                Updates LiftMovingStates state machine.
     */
    public void manualExtend(double p_power) {
        if(!Wrist.WristStates.FLAT.state) {
            super.setPower(p_power);
            lastManualTime = time;
            if (p_power != lastPower) {
                LOGGER.setLogLevel(RFLogger.Severity.INFO);
                LOGGER.log("setting power to: " + p_power);
                lastPower = p_power;
            }
        }
    }

    /**
     * Iterate to the next highest set line lift height.
     * Logs what set line lift target height the lift has been set to run to.
     * Logs to RFMotor & general logs.
     * Logs to finest level.
     * Updates LiftMovingStates state machine.
     */
    public void iterateUp() {
        for (var i : LiftMovingStates.values()) {
            if (i.state && i != LiftMovingStates.HIGH) {
                var targetState = LiftPositionStates.values()[(i.ordinal() - 1) % 4];
                setPosition(targetState);
                LOGGER.setLogLevel(RFLogger.Severity.INFO);
                LOGGER.log("iterated up to state: " + targetState);
                break;
            }
        }
    }

    /**
     * Iterate to the next lowest set line lift height.
     * Logs what set line lift target height the lift has been set to run to.
     * Logs to RFMotor & general logs.
     * Logs to finest level.
     * Updates LiftMovingStates state machine.
     */
    public void iterateDown() {
        for (var i : LiftMovingStates.values()) {
            if (i.state && i != LiftMovingStates.AT_ZERO) {
                var targetState = LiftPositionStates.values()[(i.ordinal() + 1) % 4];
                setPosition(targetState);
                LOGGER.setLogLevel(RFLogger.Severity.INFO);
                LOGGER.log("iterated down to state: " + targetState);
                break;
            }
        }
    }
}
