package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Wrist extends RFServo {
  public static double GRABBY = 1.0, DROPPY = 0.0, FLIP_TIME=0.3;
  private double lastTime=-100;
  public Wrist(){
    super("twistServo", 1.0);
    super.setPosition(GRABBY);
    super.setFlipTime(FLIP_TIME);
    lastTime = -100;
    super.setLastTime(-100);
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
    if (!p_state.state && time - lastTime > FLIP_TIME) {
      if (p_state == WristTargetStates.GRAB){
        if((Arm.ArmTargetStates.HOVER.state || Arm.ArmTargetStates.GRAB.state) && super.getPosition() != GRABBY){
          super.setPosition(GRABBY);
          LOGGER.log("wrist to GRAB");
          lastTime = time;
        }
        WristTargetStates.GRAB.setStateTrue();
      } else if(p_state == WristTargetStates.DROP){
        if((Arm.ArmTargetStates.DROP.state) && super.getPosition() != DROPPY){
          super.setPosition(DROPPY);
          LOGGER.log("wrist to DROP");
          lastTime = time;
        }
        WristTargetStates.DROP.setStateTrue();
      }
    }
    p_state.setStateTrue();
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
