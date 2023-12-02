package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Arm.ArmTargetStates.UNFLIPPED;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Wrist extends RFServo {
  double target = 0, lastTime = 0.0, FLIP_TIME = 0.4;

  public Wrist() {
    super("wristServo", 1.0);
    target = WristStates.INTAKE.position;
    flipTo(WristTargetStates.INTAKE);
    super.setPosition(WristTargetStates.INTAKE.position);
    LOGGER.log("initializing hardware");
    super.setFlipTime(FLIP_TIME);
    super.setLastTime(-100);
    lastTime=-100;
    WristTargetStates.INTAKE.setStateTrue();
    WristStates.INTAKE.setStateTrue();
  }

  public enum WristStates {
    INTAKE(0, true),
    DROP(1, false);
    double position;
    boolean state;

    WristStates(double p_position, boolean p_state) {
      position = p_position;
      state = p_state;
    }

    public void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < WristStates.values().length; i++) {
          if (WristStates.values()[i].state) {
            LOGGER.log(WristStates.values()[i].name() + " position set to: false");
          }
          WristStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log(this.name() + " position set to : true");
      }
    }
    public boolean getState(){
      return this.state;
    }
  }

  public enum WristTargetStates {
    INTAKE(0, true),
    DROP(1, false);
    double position;
    public boolean state;

    WristTargetStates(double p_position, boolean p_state) {
      position = p_position;
      state = p_state;
    }

    public void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < WristTargetStates.values().length; i++) {
          if (WristTargetStates.values()[i].state) {
            LOGGER.log(WristTargetStates.values()[i].name() + " target set to: false");
          }
          WristTargetStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log(this.name() + " target set to : true");
      }
    }
  }

  public void setIntake(){
    super.superSetPosition(1);
  }

  public void setDrop(){
    super.superSetPosition(0);
  }

  public void flipTo(WristTargetStates p_state) {
    if (target != p_state.position) {
      if (p_state == WristTargetStates.INTAKE) {
          LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
          super.setPosition(p_state.position);
          target = super.getTarget();
      }
      else if (p_state == WristTargetStates.DROP) {
        if (!Lift.LiftPositionStates.AT_ZERO.state) {
          LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
          super.setPosition(p_state.position);
          target = super.getTarget();
        }
        p_state.setStateTrue();
      }
    }
  }

  public void update() {
    for (var i : WristStates.values()) {
      if (i.position == super.getTarget() && time - super.getLastTime()> FLIP_TIME/2) {
          i.setStateTrue();
      }
      else{
      }
    }
    for (var i : WristTargetStates.values()) {
      if (i.state && super.getTarget() != i.position) {
        flipTo(i);
      }
    }
  }
}
