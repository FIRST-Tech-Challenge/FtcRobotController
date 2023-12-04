package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

@Config
public class Magazine extends RFServo {
    public static double CLOSE = 0.13, OPEN = 0.28, FLIP_TIME = 0.1;
    double lastTime = 0, target = 0;
//    private RFLEDStrip blinkin;
    private RFColorSensor colorSensor;
    boolean closed;

    public Magazine() {
        super("clawServo", 1.0);
        colorSensor = new RFColorSensor("colorSensor");
//        blinkin = new RFLEDStrip("blinkin");
//        blinkin.shotwhite();
        closed = false;
        super.setFlipTime(FLIP_TIME);
        if(isTeleop) {
            super.setPosition(OPEN);
            MagazineStates.OPEN.setStateTrue();
        }
        else {
            super.setPosition(CLOSE);
            MagazineTargetStates.CLOSE.setStateTrue();
            MagazineStates.CLOSE.setStateTrue();
        }
        super.setLastTime(-100);
        lastTime = -100;
    }

    public enum ColorStates {
        WHITE("WHITE", false),
        YELLOW("YELLOW", false),
        GREEN("GREEN", false),
        PURPLE("PURPLE", true),
        NONE("NONE",true);

        String color;
        boolean state;

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

    public enum MagazineStates {
        OPEN(1, true),
        CLOSE(0, false);
        double position;
        boolean state;

        MagazineStates(double p_position, boolean p_state) {
            position = p_position;
            state = p_state;
        }

        public void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < Magazine.MagazineStates.values().length; i++) {
                    if (Magazine.MagazineStates.values()[i].state) {
                        LOGGER.log(Magazine.MagazineStates.values()[i].name() + " position set to: false");
                    }
                    Magazine.MagazineStates.values()[i].state = false;
                }
                this.state = true;
                LOGGER.log(this.name() + " position set to : true");
            }
        }
        public boolean getState(){
            return this.state;
        }
    }

    public enum MagazineTargetStates {
        OPEN(1, true),
        CLOSE(0, false);
        double position;
        public boolean state;

        MagazineTargetStates(double p_position, boolean p_state) {
            position = p_position;
            state = p_state;
        }

        public void setStateTrue() {
            if (!this.state) {
                for (int i = 0; i < Magazine.MagazineTargetStates.values().length; i++) {
                    if (Magazine.MagazineTargetStates.values()[i].state) {
                        LOGGER.log(Magazine.MagazineTargetStates.values()[i].name() + " target set to: false");
                    }
                    Magazine.MagazineTargetStates.values()[i].state = false;
                }
                this.state = true;
                LOGGER.log(this.name() + " target set to : true");
            }
        }
    }

    public void clampTo(MagazineTargetStates p_state){
        if(target != p_state.position){
            if(p_state == MagazineTargetStates.OPEN){
                LOGGER.log("clamping to : " + p_state.name() + ", " + p_state.position);
                super.setPosition(p_state.position);
                target = super.getTarget();
            }
            else if(p_state == MagazineTargetStates.CLOSE){
                LOGGER.log("clamping to : " + p_state.name() + ", " + p_state.position);
                super.setPosition(p_state.position);
                target = super.getTarget();
            }
            p_state.setStateTrue();
        }
    }

    public void updateColor(){
        String color = colorSensor.getColor();
        for(int i = 0; i< Magazine.ColorStates.values().length; i++){
            if(ColorStates.values()[i].color.equals(color)){
                Magazine.ColorStates.values()[i].state = true;
//                if(ColorStates.values()[i].color.equals("WHITE"))
//                    blinkin.white();
//                if(ColorStates.values()[i].color.equals("GREEN"))
//                    blinkin.green();
//                if(ColorStates.values()[i].color.equals("PURPLE"))
//                    blinkin.violet();
//                if(ColorStates.values()[i].color.equals("YELLOW"))
//                    blinkin.yellow();
//                if(ColorStates.values()[i].color.equals("NONE"))
//                    blinkin.shotwhite();
            }
            else{
                Magazine.ColorStates.values()[i].state = false;
            }
        }
    }

    public void clamp() {
        LOGGER.log("closed");
        super.setPosition(CLOSE);
        if (super.getTarget()== CLOSE) {
            closed = true;
        }
    }

    public void unclamp() {
        LOGGER.log("opened");
        super.setPosition(OPEN);
        if (super.getTarget()!= OPEN) {
            closed = false;
        }
    }

    public boolean getClamped() {
        return closed;
    }

    public void update() {
        updateColor();
        for (var i : Magazine.MagazineStates.values()) {
            if (i.position == super.getTarget() && time - super.getLastTime()> FLIP_TIME/2) {
                i.setStateTrue();
            }
            else{
            }
        }
        for (var i : Magazine.MagazineTargetStates.values()) {
            if (i.state && super.getTarget() != i.position) {
                clampTo(i);
            }
        }
    }
}
