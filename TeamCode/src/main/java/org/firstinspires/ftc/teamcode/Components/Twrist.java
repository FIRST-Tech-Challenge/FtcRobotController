package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Twrist extends RFServo {
    public static double GRABBY = 1.0, DROPPY = 0.0, FLIP_TIME=0.3;
    private double lastTime=-100;
    public Twrist(){
        super("twistServo", 1.0);
        super.setPosition(GRABBY);
        super.setFlipTime(FLIP_TIME);
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
        GRAB(true, GRABBY);
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
        if (!p_state.state && time - lastTime > FLIP_TIME) {
            if (p_state == twristTargetStates.GRAB){
                if((Arm.ArmTargetStates.HOVER.state || Arm.ArmTargetStates.GRAB.state) && super.getPosition() != GRABBY){
                    super.setPosition(GRABBY);
                    LOGGER.log("twrist to GRAB");
                    lastTime = time;
                }
                twristTargetStates.GRAB.setStateTrue();
            } else if(p_state == twristTargetStates.DROP){
                if((Arm.ArmTargetStates.DROP.state) && super.getPosition() != DROPPY){
                    super.setPosition(DROPPY);
                    LOGGER.log("DROPPINg claw");
                    lastTime = time;
                }
                twristTargetStates.DROP.setStateTrue();
            }
        }
        p_state.setStateTrue();
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
