package org.firstinspires.ftc.teamcode.Components;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

/**
 * Warren
 * Class to contain all Extendo functions
 */
public class Extendo extends RFMotor {
    private boolean auto = true;
    private double targetPose = 0, manualTime = -100, UNMANUAL_TIME = 0.4;

    /**
     * Constructs extendo motor, logs to general and motor log surface level
     */
    public Extendo(){
        super("extendoMotor", true);
    }

    /**
     * Enums for preset lengths, fast for enduser input and used for state machine updating
     */
    public enum ExtendoLengths{
        FULLY_RETRACTED(0),
        HALFWAY_EXTENDED(1000),
        FULLY_EXTENDED(2000);

        final double position;
        ExtendoLengths(double p_position){
            position = p_position;
        }
    }

    /**
     * Enums for state machine, built in function for updating states
     */
    public enum ExtendoStates{
        FULLY_RETRACTED(true),
        HALFWAY_EXTENDED(false),
        FULLY_EXTENDED(false),
        EXTENDING(false),
        RETRACTING(false);

        boolean state=false;
        ExtendoStates(boolean p_state){
            state= p_state;
        }

        /**
         * Sets state machine to the current state and sets all others to false
         */
        void setStateTrue(){
           for(int i=0;i<ExtendoStates.values().length;i++){
               ExtendoStates.values()[i].state=false;
           }
            this.state=true;
        }
    }

    /**
     * Sets extendo to manual mode, auto slides turn off for UNMANUAL_TIME amount of seconds
     * logs to general and extendo log surface
     */
    public void setToManual(){
        auto=false;
        manualTime = time;
    }

    /**
     * Sets extendo to auto mode
     * logs to general and extendo log surface
     */
    public void setToAuto(){
        auto = true;
        manualTime = 0;
    }

    /**
     * Sets target position to inputted position, logs that this function is called, logs target general and motor surface level
     * @param p_position what target in ticks
     */
    public void setPosition(double p_position){
        super.setPosition(p_position,0);
        targetPose = p_position;
    }

    /**
     * Sets target position to inputted position, logs that this function is called, logs target general and motor surface level
     * @param length what target as enum preset value
     */
    public void setPosition(ExtendoLengths length){
        super.setPosition(length.position,0);
        targetPose = length.position;
    }

    /**
     * Checks if lift is busy, logs function call and comparison at finest
     * @return if lift is done moving
     */
    public boolean isDone(){
        return abs(targetPose-getCurrentPosition())<30&&abs(getVelocity())<30;
    }

    /**
     * updates the state machine and powers of the slide if in auto mode, logs if state changed(inside the enum function definition), logs if manual changed
     */
    public void update(){
        if(time-manualTime>UNMANUAL_TIME){
            auto=true;
        }
        if(auto){
            super.setPosition(targetPose, 0);
        }
        if(abs(targetPose-getCurrentPosition())<30&&abs(getVelocity())<30){
            if(targetPose == ExtendoLengths.FULLY_RETRACTED.position)
                ExtendoStates.FULLY_RETRACTED.setStateTrue();
            else if(targetPose == ExtendoLengths.HALFWAY_EXTENDED.position)
                ExtendoStates.FULLY_RETRACTED.setStateTrue();
            else if(targetPose == ExtendoLengths.FULLY_EXTENDED.position)
                ExtendoStates.FULLY_RETRACTED.setStateTrue();
        }


    }
}
