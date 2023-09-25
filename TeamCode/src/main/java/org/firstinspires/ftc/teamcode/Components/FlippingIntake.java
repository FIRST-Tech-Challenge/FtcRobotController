package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFBreakBeam;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLimitSwitch;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Warren
 * Class to contain flipping intake and associated functions
 */
public class FlippingIntake {
    RFMotor intake;
    RFServo flipper;
    RFBreakBeam breakBeam;
    RFLimitSwitch limitSwitch;

    private final double FLIP_TIME = 0.4;
    private final double INTAKE_POWER = 0.6;
    private final double REVERSE_POWER=-0.3;

    /**
     * initializes all the hardware, logs that hardware has been initialized
     */
    public FlippingIntake(){
        intake = new RFMotor("intakeMotor", true);
        flipper = new RFServo("flipServo",1);
        breakBeam = new RFBreakBeam("breakBeam");
        limitSwitch = new RFLimitSwitch("intakeSwitch");
    }

    /**
     * possible states of intake
     */
    public enum FlintakeStates{
        EMPTY_DOWN(true),
        HALF_DOWN(false),
        FULL_DOWN(false),
        FULL_MID(false),
        FULL_UP(false),
        FLIPPING_UP(false),
        FLIPPING_DOWN(false),
        EMPTY_UP(false);
        boolean state;
        FlintakeStates(boolean p_state){
            state = p_state;
        }

        /**
         * sets current state to true, logs that this state is true in general and intake surface
         */
        public void setStateTrue() {
            for(int i=0;i<FlintakeStates.values().length;i++){
                FlintakeStates.values()[i].state=false;
            }
            this.state = true;
        }
    }

    /**
     * enum to contain preset positions of flipper
     */
    public enum FlintakePos{
        DOWN(0.0),
        MID(0.5),
        UP(1.0);
        final double position;
        FlintakePos(double p_position){
            position = p_position;
        }
    }
    public void intake(){
        intake.setPower(INTAKE_POWER);
    }
    public void reverseIntake(){
        intake.setPower(REVERSE_POWER);
    }
    public void stopIntake(){
        intake.setPower(0);
    }
    public void setIntakePower(double p_power){
        intake.setPower(p_power);
    }
    public void flipUp(){
        flipper.setPosition(FlintakePos.UP.position);
    }
    public void flipDown(){
        flipper.setPosition(FlintakePos.DOWN.position);
    }
    public void flipMid(){
        flipper.setPosition(FlintakePos.MID.position);
    }
    public void flipTo(double p_pos){
        flipper.setPosition(p_pos);
    }

    /**
     * count number of pixels using sensor, log to intake and general surface level when change
     * @return number of pixels in intake
     */
    public int countPixels(){
        int count=0;
        if(limitSwitch.isSwitched()){
            count++;
//            break beam here
            if(true){
                count++;
            }
        }
        return count;
    }


    /**
     * updates the state machine, log in general and intake surface
     * updates sensor information, triggers following action
     */
    public void update(){

    }
}
