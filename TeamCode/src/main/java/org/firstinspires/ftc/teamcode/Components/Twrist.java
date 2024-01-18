package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
@Config
public class Twrist extends RFServo {
    public static double GRABBY = 0.585, DROPPY = 0.113, FLIP_TIME=0.3;
    private double lastTime=-100;
    public Twrist(){
        super("twistServo", 1.0);
        lastTime = -100;
        super.setLastTime(-100);
        super.setPosition(GRABBY);
        super.setFlipTime(FLIP_TIME);
        twristStates.GRAB.setStateTrue();
        twristTargetStates.GRAB.setStateTrue();
        twristTargetStates.GRAB.state = false;
        lastTime = -100;
        super.setLastTime(-100);
    }
    public enum twristStates{
        DROP(false, DROPPY),
        GRAB(true, GRABBY);
        boolean state;
        double pos;
        twristStates(boolean p_state, double p_pos){
            state=p_state;
            pos = p_pos;
        }
        public boolean getState(){
            return state;
        }
        public void setStateTrue(){
            LOGGER.log("twrist state set to : " + this.name());
            for(var i : Twrist.twristStates.values()){
                i.state=i==this;
            }
        }
    }
    public enum twristTargetStates{
        DROP(false, DROPPY),
        GRAB(false, GRABBY);
        boolean state;
        double pos;
        twristTargetStates(boolean p_state, double p_pos){
            state=p_state;
            pos = p_pos;
        }
        public boolean getState(){
            return state;
        }
        public void setStateTrue(){ 
            LOGGER.log("twristTarget state set to : " + this.name());
            for(var i : Twrist.twristTargetStates.values()){
                i.state=i==this;
            }
        }
    }
    public void flipTo(twristTargetStates p_state){
        if (!twristStates.values()[p_state.ordinal()].state&& abs(time - lastTime) > FLIP_TIME) {
            if (p_state == twristTargetStates.GRAB){
                if((Arm.ArmStates.HOVER.state || Arm.ArmStates.GRAB.state) && super.getPosition() != GRABBY){
                    super.setPosition(GRABBY);
                    LOGGER.log("twrist to GRAB");
                    lastTime = time;
                }
                twristTargetStates.GRAB.setStateTrue();
            } else if(p_state == twristTargetStates.DROP){
                if((Arm.ArmStates.DROP.state) && super.getPosition() != DROPPY){
                    super.setPosition(DROPPY);
                    LOGGER.log("twrist to DROP");
                    lastTime = time;
                }
                twristTargetStates.DROP.setStateTrue();
            }
        }
        p_state.state=true;
    }
    public void update() {
        for (var i : twristStates.values()) {
            if (super.getPosition() == i.pos && time > lastTime + FLIP_TIME) i.setStateTrue();
        }
        for (var i : twristTargetStates.values()) {
            if (i.state && super.getPosition() != i.pos) flipTo(i);
        }
    }
}
