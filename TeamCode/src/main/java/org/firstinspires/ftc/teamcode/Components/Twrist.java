package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
@Config
public class Twrist extends RFServo {
    public static double GRABBY = 0.58, LEFT_TILTY = 0.4, RIGHT_TILTY = 0.7, DROPPY = 0.113, FLIP_TIME=0.3;
    private double lastTime=-100;
    public Twrist(){
        super("twistServo", 1.0);
        lastTime = -100;
        super.setLastTime(-100);
        super.setPosition(GRABBY);
        super.setFlipTime(FLIP_TIME);
        twristStates.GRAB.setStateTrue();
        twristTargetStates.GRAB.setStateTrue();
    }
    public enum twristStates{
        DROP(false, DROPPY),
        GRAB(true, GRABBY),
        LEFT_TILT(false, LEFT_TILTY),
        RIGHT_TILT(false, RIGHT_TILTY);

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
        GRAB(true, GRABBY),
        LEFT_TILT(false, LEFT_TILTY),
        RIGHT_TILT(false, RIGHT_TILTY);
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
        op.telemetry.addData("ok", !twristStates.values()[p_state.ordinal()].state);
        if (!twristStates.values()[p_state.ordinal()].state&& abs(time - lastTime) > FLIP_TIME) {
            if (p_state == twristTargetStates.GRAB){
                op.telemetry.addData("hi", "hi1");
                if((Arm.ArmTargetStates.HOVER.state || Arm.ArmTargetStates.GRAB.state) && super.getPosition() != GRABBY){
                    super.setPosition(GRABBY);
                    LOGGER.log("twrist to GRAB");
                    lastTime = time;
                }
                twristTargetStates.GRAB.setStateTrue();
            } else if(p_state == twristTargetStates.DROP){
                op.telemetry.addData("hi", "hi2");

                if((Arm.ArmStates.DROP.state) && super.getPosition() != DROPPY){
                    super.setPosition(DROPPY);
                    LOGGER.log("twrist to DROP");
                    lastTime = time;
                }
                twristTargetStates.DROP.setStateTrue();
            }
            else if(p_state == twristTargetStates.LEFT_TILT) {
                op.telemetry.addData("hi", "hi3");

                if((Arm.ArmStates.DROP.state) && super.getPosition() != LEFT_TILTY){
                    super.setPosition(LEFT_TILTY);
                    LOGGER.log("left tilting claw");
                    lastTime = time;
                }
                twristTargetStates.LEFT_TILT.setStateTrue();
            }
            else if(p_state == twristTargetStates.RIGHT_TILT) {
                op.telemetry.addData("hi", "hi4");

                if((Arm.ArmStates.DROP.state) && super.getPosition() != RIGHT_TILTY){
                    super.setPosition(RIGHT_TILTY);
                    LOGGER.log("right tilting claw");
                    lastTime = time;
                }
                twristTargetStates.RIGHT_TILT.setStateTrue();
            }
        }
        op.telemetry.update();
        p_state.state=true;
    }
    public void update() {
        for (var i : twristStates.values()) {
            if (super.getPosition() == i.pos && time +0.1> lastTime + FLIP_TIME) i.setStateTrue(); twristTargetStates.values()[i.ordinal()].state=false;
        }
        for (var i : twristTargetStates.values()) {
            if (i.state && super.getPosition() != i.pos) flipTo(i);
        }
    }
}
