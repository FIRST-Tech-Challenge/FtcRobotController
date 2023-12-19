package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Magazine.pixels;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

@Config
public class Claw extends RFServo {
  public static double DROP_POS = 1.0,
      GRAB_POS = 0.4,
      CLOSE_POS = 0.3,
      PINCH_POS = 0.2,
      FLIP_TIME = 0.2;
  private double lastTime = 0;

  public Claw() {
    super("clawServo", 1.0);
    super.setPosition(PINCH_POS);
    super.setFlipTime(FLIP_TIME);
    lastTime = -100;
    super.setLastTime(-100);
  }

  public enum clawStates {
    DROP(false, DROP_POS),
    GRAB(false, GRAB_POS),
    CLOSE(false, CLOSE_POS),
    PINCH(true, PINCH_POS);
    boolean state;
    double pos;

    clawStates(boolean p_state, double p_pos) {
      state = p_state;
      pos = p_pos;
    }

    public boolean getState() {
      return state;
    }

    public void setStateTrue() {
      LOGGER.log("CLAW state set to : " + this.name());
      for (var i : clawStates.values()) {
        i.state = i == this;
      }
    }
  }

  public enum clawTargetStates {
    DROP(false, DROP_POS),
    GRAB(false, GRAB_POS),
    CLOSE(false, CLOSE_POS),
    PINCH(true, PINCH_POS);
    boolean state;
    double pos;

    clawTargetStates(boolean p_state, double p_pos) {
      state = p_state;
      pos = p_pos;
    }

    public boolean getState() {
      return state;
    }

    public void setStateTrue() {
      LOGGER.log("CLAW_TARGET state set to : " + this.name());
      for (var i : clawTargetStates.values()) {
        i.state = i == this;
      }
    }
  }

  public void flipTo(clawTargetStates p_state) {
    if (!p_state.state && time - lastTime > FLIP_TIME) {
      if (p_state == clawTargetStates.CLOSE || p_state == clawTargetStates.PINCH) {
        if (Arm.ArmStates.GRAB.state) {
          if (pixels == 2) {
            if (super.getPosition() != CLOSE_POS) {
              super.setPosition(CLOSE_POS);
              LOGGER.log(RFLogger.Severity.INFO, "CLOSING claw");
              lastTime = time;
            }
            clawTargetStates.CLOSE.setStateTrue();
          } else if (pixels == 1) {
            if (p_state == clawTargetStates.PINCH && super.getPosition() != PINCH_POS) {
              super.setPosition(PINCH_POS);
              LOGGER.log(RFLogger.Severity.INFO, "PINCHING claw");
              lastTime = time;
            }
            clawTargetStates.PINCH.setStateTrue();
          } else if (super.getPosition() != PINCH_POS && super.getPosition() != CLOSE_POS) {
            clawTargetStates.GRAB.setStateTrue();
          }
        } else {
          Arm.ArmTargetStates.GRAB.setStateTrue();
        }
      } else if (p_state == clawTargetStates.GRAB) {
        if((Arm.ArmTargetStates.HOVER.state || Arm.ArmTargetStates.GRAB.state) && super.getPosition() != GRAB_POS){
          super.setPosition(GRAB_POS);
          LOGGER.log("GRABBING claw");
          lastTime = time;
        }
        clawTargetStates.GRAB.setStateTrue();
      } else if(p_state == clawTargetStates.DROP){
        if((Arm.ArmStates.DROP.state) && super.getPosition() != DROP_POS){
          super.setPosition(DROP_POS);
          LOGGER.log("DROPPINg claw");
          lastTime = time;
        }
        clawTargetStates.DROP.setStateTrue();
      }
    }}

  public void update() {
    for (var i : clawStates.values()) {
      if (super.getPosition() == i.pos && time > lastTime + FLIP_TIME) i.setStateTrue();
    }
    for (var i : clawTargetStates.values()) {
      if (i.state && super.getPosition() != i.pos) flipTo(i);
    }
  }
}
