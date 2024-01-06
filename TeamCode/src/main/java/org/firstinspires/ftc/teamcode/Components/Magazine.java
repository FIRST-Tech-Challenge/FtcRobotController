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
    private RFLEDStrip blinkin;
    private RFColorSensor colorSensor, colorSensor2;
    boolean closed;

    int colored = 0;
    public static int pixels = 0;

    public Magazine() {
        super("magServo", 1.0);
        colorSensor = new RFColorSensor("colorSensor");
        colorSensor2 = new RFColorSensor("colorSensor2");
        blinkin = new RFLEDStrip("blinkin");
        blinkin.shotwhite();
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
      if (p_state == MagazineTargetStates.OPEN && !Intake.IntakeStates.INTAKING.getState()) {
                LOGGER.log("clamping to : " + p_state.name() + ", " + p_state.position);
                super.setPosition(p_state.position);
                target = super.getTarget();
                MagazineTargetStates.OPEN.setStateTrue();
                lastTime=time;
      } else if (p_state == MagazineTargetStates.CLOSE && !Arm.ArmTargetStates.GRAB.state) {
                LOGGER.log("clamping to : " + p_state.name() + ", " + p_state.position);
                super.setPosition(p_state.position);
                MagazineTargetStates.CLOSE.setStateTrue();
                target = super.getTarget();
                lastTime = time;
            }
            p_state.setStateTrue();
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
    }
    public void updateBlinkin(){
        pixels=0;
        if(!ColorStates.GREEN.getTrue().equals("NONE"))pixels++;
        if(!ColorStates2.GREEN.getTrue().equals("NONE"))pixels++;
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
        for (var i : Magazine.MagazineStates.values()) {
            if (i.position == super.getTarget() && time - super.getLastTime()> FLIP_TIME) {
                i.setStateTrue();
            }
        }
        for (var i : Magazine.MagazineTargetStates.values()) {
            if (i.state && super.getTarget() != i.position) {
                clampTo(i);
            }
        }
    }
}
