package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

@Config
public class Magazine {
    public static double CLOSE = 0.13, OPEN = 0.28, FLIP_TIME = 0.1;
    double lastTime = 0, target = 0;
    private RFLEDStrip blinkin;
    private RFColorSensor colorSensor, colorSensor2;
    boolean closed;

    int colored = 0;
    public static int pixels = 0;

    public Magazine() {
        colorSensor = new RFColorSensor("colorSensor");
        colorSensor2 = new RFColorSensor("colorSensor2");
//        blinkin = new RFLEDStrip("blinkin");
//        blinkin.shotwhite();
    }

    public enum ColorStates {
        WHITE("WHITE", false),
        YELLOW("YELLOW", false),
        GREEN("GREEN", false),
        PURPLE("PURPLE", true),
        NONE("NONE",true);

        String color;
        private boolean state;

        ColorStates(String p_color, boolean p_state) {
            this.color = p_color;
            this.state = p_state;
        }

        void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < Magazine.ColorStates.values().length; i++) {
                    Magazine.ColorStates.values()[i].state = false;
                }
                this.state = true;
            }
        }

        public String getTrue(){
            for(int i = 0; i< Magazine.ColorStates.values().length; i++){
                if(ColorStates.values()[i].state){
                    return ColorStates.values()[i].color;
                }
            }
            return NONE.color;
        }

        public boolean getState() {
            return this.state;
        }

        public String getColor() {
            return this.color;
        }
    }
    public enum ColorStates2 {
        WHITE("WHITE", false),
        YELLOW("YELLOW", false),
        GREEN("GREEN", false),
        PURPLE("PURPLE", true),
        NONE("NONE",true);

        String color;
        boolean state;

        ColorStates2(String p_color, boolean p_state) {
            this.color = p_color;
            this.state = p_state;
        }

        void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < Magazine.ColorStates.values().length; i++) {
                    Magazine.ColorStates.values()[i].state = false;
                }
                this.state = true;
            }
        }

        public String getTrue(){
            for(int i = 0; i< Magazine.ColorStates.values().length; i++){
                if(ColorStates.values()[i].state){
                    return ColorStates.values()[i].color;
                }
            }
            return NONE.color;
        }

        public boolean getState() {
            return this.state;
        }

        public String getColor() {
            return this.color;
        }
    }

    public void updateColor(){
        String color = colorSensor.getColor();
        for(int i = 0; i< Magazine.ColorStates.values().length; i++){
            if(ColorStates.values()[i].color.equals(color)){
                Magazine.ColorStates.values()[i].state = true;
            }
            else{
                Magazine.ColorStates.values()[i].state = false;
            }
        }
        String color2 = colorSensor2.getColor();
        for(int i = 0; i< Magazine.ColorStates2.values().length; i++){
            if(ColorStates2.values()[i].color.equals(color2)){
                Magazine.ColorStates2.values()[i].state = true;
            }
            else{
                Magazine.ColorStates.values()[i].state = false;
            }
        }
        pixels=0;
        if(colorSensor.getDist()<1) pixels++;
        if(colorSensor2.getDist()<1) pixels++;
    }
    public void updateBlinkin(){
        if(time%3<1 && colored!=0){
            if(pixels==0)
                blinkin.red();
            else if(pixels==1)
                blinkin.orange();
            else
                blinkin.bluegreen();
            colored=0;
        }
        else if(time%3<2&&colored!=1){
            if(ColorStates.GREEN.getTrue().equals("WHITE"))
                blinkin.white();
            if(ColorStates.GREEN.getTrue().equals("GREEN"))
                blinkin.green();
            if(ColorStates.GREEN.getTrue().equals("PURPLE"))
                blinkin.violet();
            if(ColorStates.GREEN.getTrue().equals("YELLOW"))
                blinkin.yellow();
            if(ColorStates.GREEN.getTrue().equals("NONE"))
                blinkin.shotwhite();
            colored=1;
        }
        else if(colored!=2){
            if(ColorStates2.GREEN.getTrue().equals("WHITE"))
                blinkin.white();
            if(ColorStates2.GREEN.getTrue().equals("GREEN"))
                blinkin.green();
            if(ColorStates2.GREEN.getTrue().equals("PURPLE"))
                blinkin.violet();
            if(ColorStates2.GREEN.getTrue().equals("YELLOW"))
                blinkin.yellow();
            if(ColorStates2.GREEN.getTrue().equals("NONE"))
                blinkin.shotwhite();
            colored=2;
        }
    }
    public int getPixels(){
        return pixels;
    }


    public boolean getClamped() {
        return closed;
    }

    public void update() {
        updateColor();
        updateBlinkin();
    }
}
