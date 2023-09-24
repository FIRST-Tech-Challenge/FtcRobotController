package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Harry
 */
public class Hopper extends RFServo {
    RFLEDStrip leds;
    RFColorSensor colorSensor;
    public Hopper(){
        super("hopperServo", 0);
        leds = new RFLEDStrip();
        colorSensor = new RFColorSensor("colorSensor");
    }
    public enum HopperValues{
        ZEROPIXEL(0),
        ONEPIXEL(0.5),
        TWOPIXEL(1);

        final double target;
        HopperValues(double p_target){
            target = p_target;
        }
    }

    public enum HopperColor{
        WHITE(false),
        PURPLE(false),
        YELLOW(false),
        GREEN(false),
        NONE(true);

        boolean state = false;
        HopperColor(boolean p_state){
            state = p_state;
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
        return "";
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
