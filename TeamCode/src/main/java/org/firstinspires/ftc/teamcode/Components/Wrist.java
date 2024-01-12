package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
@Config
public class Wrist extends RFServo {
  public static double GRABBY = 0.15, DROPPY = 0.3, FLIP_TIME=0.3;
  private double lastTime=-100;
  public Wrist(){
    super("wristServo", 1.0);
    lastTime = -100;
    super.setLastTime(-100);
    super.setPosition(GRABBY);
    super.setFlipTime(FLIP_TIME);
    WristStates.GRAB.setStateTrue();
    WristTargetStates.GRAB.setStateTrue();
  }
  public enum WristStates{
    DROP(false, DROPPY),
    GRAB(true, GRABBY);
    boolean state;
    double pos;
    WristStates(boolean p_state, double p_pos){
      state=p_state;
      pos = p_pos;
    }
    public boolean getState(){
      return state;
    }
    public void setStateTrue(){
      if(!this.state)
        LOGGER.log("wrist state set to : " + this.name());
      for(var i : WristStates.values()){
        i.state=i==this;
      }
    }
  }
  public enum WristTargetStates{
    DROP(false, DROPPY),
    GRAB(true, GRABBY);
    boolean state;
    double pos;
    WristTargetStates(boolean p_state, double p_pos){
      state=p_state;
      pos = p_pos;
    }
    public boolean getState(){
      return state;
    }
    public void setStateTrue(){
      LOGGER.log("wristTarget state set to : " + this.name());
      for(var i : WristTargetStates.values()){
        i.state=i==this;
      }
    }
  }
  public void flipTo(WristTargetStates p_state){
    if (!WristStates.values()[p_state.ordinal()].state&& abs(time - lastTime) > FLIP_TIME) {
      if (p_state == WristTargetStates.GRAB){
        if((Arm.ArmStates.HOVER.state || Arm.ArmStates.GRAB.state) && super.getPosition() != GRABBY){
          super.setPosition(GRABBY);
          LOGGER.log("wrist to GRAB");
          lastTime = time;
        }
        WristTargetStates.GRAB.setStateTrue();
      } else if(p_state == WristTargetStates.DROP){
        if((Arm.ArmStates.DROP.state) && super.getPosition() != DROPPY){
          super.setPosition(DROPPY);
          LOGGER.log("wrist to DROP");
          lastTime = time;
        }
        WristTargetStates.DROP.setStateTrue();
      }
    }
    p_state.state = true;
  }
  public void update() {
    for (var i : WristStates.values()) {
      if (super.getPosition() == i.pos && time > lastTime + FLIP_TIME) i.setStateTrue();
    }
    for (var i : WristTargetStates.values()) {
      if (i.state && super.getPosition() != i.pos) flipTo(i);
    }
  }
}
