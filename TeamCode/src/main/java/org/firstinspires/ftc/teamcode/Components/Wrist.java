package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Arm.ArmTargetStates.UNFLIPPED;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Wrist extends RFServo {
  double target = 0, lastTime = 0.0, FLIP_TIME = 0.2;

  public Wrist() {
    super("wristServo", 1.0);
    target = WristStates.FLAT.position;
    flipTo(WristTargetStates.FLAT);
    LOGGER.log("initializing hardware");
    super.setLastTime(-100);
    lastTime=-100;
  }

  public enum WristStates {
    FLAT(1.0, true),
    HOLD(0.55, false),
    FLIP(0.21, false),

    DROP(0.0, false);
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
  }

  public enum WristTargetStates {
    FLAT(1.0, true),
    FLIP(0.23, false),

    HOLD(0.55, false),
    DROP(0.0, false);
    double position;
    boolean state;

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

  public void flipTo(WristTargetStates p_state) {
    if (target != p_state.position) {
      if (p_state == WristTargetStates.FLAT) {
        if (Arm.ArmStates.UNFLIPPED.state
            && Lift.LiftPositionStates.AT_ZERO.state) {
          LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
          super.setPosition(p_state.position);
          target = super.getTarget();
        } else if (Arm.ArmStates.UNFLIPPED.state) {
          LOGGER.log(
              "flipping to : "
                  + WristTargetStates.HOLD.name()
                  + ", "
                  + WristTargetStates.HOLD.position);
          super.setPosition(WristTargetStates.HOLD.position);
          target = super.getTarget();
        } else {
          LOGGER.log("slides not in right position, can't flip to flat");
        }
      } else if (p_state == WristTargetStates.HOLD) {
        if (Arm.ArmStates.UNFLIPPED.state) {
          LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
          super.setPosition(p_state.position);
          target = super.getTarget();
        } else LOGGER.log("arm not in right position, can't flip to hold");
      } else if (p_state == WristTargetStates.DROP) {
        if (!Lift.LiftPositionStates.AT_ZERO.state) {
          LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
          super.setPosition(p_state.position);
          target = super.getTarget();
        } else LOGGER.log("lift not in right position, can't flip to drop");
      } else if (p_state == WristTargetStates.FLIP) {
        if (!Lift.LiftPositionStates.AT_ZERO.state) {
          LOGGER.log("flipping to : " + p_state.name() + ", " + p_state.position);
          super.setPosition(p_state.position);
          target = super.getTarget();
        } else LOGGER.log("lift not in right position, can't flip to flip");
      }
      p_state.setStateTrue();
    }
  }

  public void update() {
    for (var i : WristStates.values()) {
      if (i.position == target && time - lastTime > FLIP_TIME) {
        i.setStateTrue();
      }
    }
    for (var i : WristTargetStates.values()) {
      if (i.state && target != i.position) {
        flipTo(i);
      }
    }
  }
}
