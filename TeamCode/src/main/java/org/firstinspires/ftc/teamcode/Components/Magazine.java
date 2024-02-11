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
    private RFLEDStrip blinkin;
    private RFColorSensor colorSensor1, colorSensor2;

    public static int pixels = 0;

    public Magazine() {
        colorSensor1 = new RFColorSensor("colorSensor");
        colorSensor2 = new RFColorSensor("colorSensor2");
        blinkin = new RFLEDStrip("blinkin");
    }

    public enum MagStates {
        FRONT(false),
        BACK(false);

        private boolean state;
        MagStates(boolean p_state) {
            this.state = p_state;
        }

        void toggle() {
            this.state = !this.state;
        }

        void setState(boolean c_state){
            this.state = c_state;
        }

        public boolean getState() {
            return this.state;
        }
    }

    public double frontdist(){
        return colorSensor1.getDist();
    }

    public double backdist(){
        return colorSensor2.getDist();
    }

    public void updateSensors(){
        double dist1 = colorSensor1.getDist();
        double dist2 = colorSensor2.getDist();
        if(dist1 < 1){
            MagStates.FRONT.setState(true);
        }
        else if(dist1 > 1){
            MagStates.FRONT.setState(false);
        }
        if(dist2 <1){
            MagStates.BACK.setState(true);
        }
        else if(dist2 > 1){
            MagStates.BACK.setState(false);
        }
    }

    public void updatePixels(){
        if(MagStates.FRONT.state && MagStates.BACK.state){
            pixels = 2;
        }
        else if(!MagStates.FRONT.state && !MagStates.BACK.state){
            pixels = 0;
        }
        else{
            pixels = 1;
        }
    }

    public void updateBlinkin(){
        if(Arm.ArmStates.HOVER.state && Claw.clawStates.GRAB.state){
            blinkin.green();
        }
        else if(Arm.ArmStates.GRAB.state){
            blinkin.yellow();
        }
        else if(pixels == 0){
            blinkin.red();
        }
        else if(pixels == 1){
            blinkin.redorange();
        }
        else if(pixels == 2){
            blinkin.white();
        }
    }

    public int getPixels(){
        return pixels;
    }

    public void update() {
        LOGGER.log("front | back dist: " + frontdist() + " | " + backdist());
        LOGGER.log("front | back state: " + MagStates.FRONT.getState() + " | " + MagStates.BACK.getState());
        LOGGER.log("# Pixels: " + getPixels());
        updateSensors();
        updatePixels();
        updateBlinkin();
    }
}
