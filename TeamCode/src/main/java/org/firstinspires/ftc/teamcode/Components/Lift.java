package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

/**
 * William
 */
public class Lift extends RFMotor {
    public Lift() {
        super("liftMotor", true);
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
        void setState(boolean p_state) {
            this.state = p_state;
        }
    }

    public enum LiftMovingStates {
        AT_ZERO(true),
        LOW(false),
        MID(false),
        HIGH(false),
        LIFTING(false),
        LOWERING(false);

        boolean state;

        LiftMovingStates(boolean p_state) {
            this.state = p_state;
        }
        void setState(boolean p_state) {
            this.state = p_state;
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

    }

    /**
     * Sets target position for lift.
     * @param p_target target position for lift to run to
     * Logs what position the target position has been set to.
     * Logs to RFMotor & general logs.
     * Logs to finest level.
     * Updates LiftMovingStates state machine.
     */
    public void setPosition(double p_target) {
        super.setPosition(p_target, 0);
    }
    public void setPosition(LiftPositionStates p_state) {
        super.setPosition(p_state.position, 0);
    }

    /**
     * Manually extend/retract slides
     * @param power How fast the user wants to move the slides and in what direction
     * Logs that the lift is currently being manually extended.
     * Logs to RFMotor & general logs.
     * Logs to finest level.
     * Updates LiftMovingStates state machine.
     */
    public void setManual(double power) {

    }

    /**
     * Iterate to the next highest set line lift height.
     * Logs what set line lift target height the lift has been set to run to.
     * Logs to RFMotor & general logs.
     * Logs to finest level.
     * Updates LiftMovingStates state machine.
     */
    public void iterateUp() {

    }

    /**
     * Iterate to the next lowest set line lift height.
     * Logs what set line lift target height the lift has been set to run to.
     * Logs to RFMotor & general logs.
     * Logs to finest level.
     * Updates LiftMovingStates state machine.
     */
    public void iterateDown() {

    }
}
