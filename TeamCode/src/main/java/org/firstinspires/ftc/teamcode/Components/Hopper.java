package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Harry
 * Class to contain all Hopper functions
 */
public class Hopper extends RFServo {
    private final static double ZERO_POSITION = 0.0, ONE_POSITION = 0.5, TWO_POSITION = 1.0;
    RFLEDStrip leds;
    RFColorSensor colorSensor;

    /**
     * Constructs Hopper servo and leds and colorsensor, log to general with config severity
     */
    public Hopper(){
        super("hopperServo", 1.0);
//        leds = new RFLEDStrip();
//        colorSensor = new RFColorSensor("colorSensor");
    }

    /**
     * enum for hopper values, used when setting how many pixels to outtake
     */
    public enum HopperValues{
        ZEROPIXEL(ZERO_POSITION),
        ONEPIXEL(ONE_POSITION),
        TWOPIXEL(TWO_POSITION);

        final double target;
        HopperValues(double p_target){
            target = p_target;
        }
    }

    /**
     * enum for hopper colors, built in function for setting states
     */
    public enum HopperColor{
        WHITE(false, "WHITE"),
        PURPLE(false, "PURPLE"),
        YELLOW(false, "YELLOW"),
        GREEN(false, "GREEN"),
        NONE(true, "NONE");

        boolean state = false;
        String color = "";
        HopperColor(boolean p_state, String p_color){
            state = p_state;
            color = p_color;
        }

        void setStateTrue(){
            for(int i = 0; i < HopperColor.values().length; i++){
                HopperColor.values()[i].state = false;
            }
            this.state = true;
        }

        HopperColor getTrueState(){
            for(int i = 0; i < HopperColor.values().length; i++){
                if(HopperColor.values()[i].state){
                    return HopperColor.values()[i];
                }
            }
            return null;
        }
    }

    /**
     * enum for hopper states, built in function for updating states
     */
    public enum HopperStates{
        ZERO(true),
        ONE(false),
        TWO(false);

        boolean state = false;
        HopperStates(boolean p_state){
            state = p_state;
        }

        void setStateTrue(){
            for(int i = 0; i < HopperStates.values().length; i++){
                HopperStates.values()[i].state = false;
            }
            this.state = true;
        }

        public boolean getState(){
            return this.state;
        }
    }

    /**
     * void, returns a string of the pixel color in all capital letters, log to general with INFO severity
     * @return
     */
    public String getPixelColor(){
        return HopperColor.NONE.getTrueState().color;
    }

    /**
     * void, sets the servo position to outtake 1 or 2 or 0 pixels, log to general with highest verbosity
     * @param p_pixelNumber = how many to outtake
     */
    public void outtakePixel(HopperValues p_pixelNumber){
        super.setPosition(p_pixelNumber.target);
    }

    /**
     * void, updates the states of where servo is at now, log to general inside enum function
     */
    public void update(){
        if(super.getPosition() == 0){
            HopperStates.ZERO.setStateTrue();
        }
        else if(super.getPosition() == 0.5){
            HopperStates.ONE.setStateTrue();
        }
        else if(super.getPosition() == 1){
            HopperStates.TWO.setStateTrue();
        }
    }
}
