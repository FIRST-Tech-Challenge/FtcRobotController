package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Harry
 * Class to contain all Arm functions
 */
public class Arm extends RFServo {
    private final double LOWER_LIMIT = 0.0, UPPER_LIMIT = 1.0;

    /**
     * constructs arm servo, logs to general with CONFIG severity
     */
    public Arm() {
        super("armServo", 1.0);
    }

    /**
     * enum for arm servo states, built in function to update states
     */
    public enum ArmStates{
        UNFLIPPED(true),
        FLIPPED(false);

        boolean state = false;
        ArmStates(boolean p_state){
            state = p_state;
        }

        void setStateTrue(){
            for(int i = 0; i < Arm.ArmStates.values().length; i++){
                Arm.ArmStates.values()[i].state = false;
            }
            this.state = true;
        }
    }

    /**
     * void, toggles arm between outtake position and intake position
     * logs to general with highest verbosity
     */
    public void flip(){
        if(ArmStates.UNFLIPPED.state){
            super.setPosition(UPPER_LIMIT);
        }
        else{
            super.setPosition(LOWER_LIMIT);
        }
    }

    /**
     * void, sets arm servo position to input position
     * logs to general with highest verbosity
     * @param p_target target position
     */
    public void setPosition(double p_target){
        super.setPosition(p_target);
    }

    /**
     * void, updates state machine according to where servo is
     * logs to general inside the enum functions
     */
    public void update() {
        if(super.getPosition() == 0){
            ArmStates.UNFLIPPED.setStateTrue();
        }
        else if(super.getPosition() == 1){
            ArmStates.FLIPPED.setStateTrue();
        }
    }
}
