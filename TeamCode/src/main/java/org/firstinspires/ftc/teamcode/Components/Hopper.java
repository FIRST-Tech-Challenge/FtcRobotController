package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Harry
 */
public class Hopper extends RFServo {
    private final static double ZERO_POSITION = 0.0, ONE_POSITION = 0.5, TWO_POSITION = 1.0;
    RFLEDStrip leds;
    RFColorSensor colorSensor;
    public Hopper(){
        super("hopperServo", 1.0);
        leds = new RFLEDStrip();
        colorSensor = new RFColorSensor("colorSensor");
    }
    public enum HopperValues{
        ZEROPIXEL(ZERO_POSITION),
        ONEPIXEL(ONE_POSITION),
        TWOPIXEL(TWO_POSITION);

        final double target;
        HopperValues(double p_target){
            target = p_target;
        }
    }

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
    }

    public String getPixelColor(){
        return HopperColor.NONE.getTrueState().color;
    }

    public void outtakePixel(HopperValues p_pixelNumber){
        super.setPosition(p_pixelNumber.target);
    }

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
