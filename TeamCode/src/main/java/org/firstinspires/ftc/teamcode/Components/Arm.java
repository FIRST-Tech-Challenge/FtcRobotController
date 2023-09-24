package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Harry
 */
public class Arm extends RFServo {
    private final double LOWER_LIMIT = 0.0, UPPER_LIMIT = 1.0;
    public Arm() {
        super("armServo", 1.0);
    }

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

    public void flip(){
        if(ArmStates.UNFLIPPED.state){
            super.setPosition(UPPER_LIMIT);
        }
        else{
            super.setPosition(LOWER_LIMIT);
        }
    }

    public void setPosition(double p_target){
        super.setPosition(p_target);
    }

    public void update() {
        if(super.getPosition() == 0){
            ArmStates.UNFLIPPED.setStateTrue();
        }
        else if(super.getPosition() == 1){
            ArmStates.FLIPPED.setStateTrue();
        }
    }
}
