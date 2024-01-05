package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Magazine.pixels;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

@Config
public class Claw extends RFServo {
  public static double GRAB_POS = 0.0,
      CLOSE_POS = 0.2,
      FLIP_TIME = 0.1;
  private double lastTime = 0;

  public Claw() {
    super("clawServo", 1.0);
    lastTime = -100;
    super.setLastTime(-100);
    super.setPosition(GRAB_POS);
    super.setFlipTime(FLIP_TIME);
    clawStates.GRAB.setStateTrue();
    clawTargetStates.GRAB.setStateTrue();
  }

  public enum clawStates {
    GRAB(false, GRAB_POS),
    CLOSE(false, CLOSE_POS);
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
    GRAB(false, GRAB_POS),
    CLOSE(false, CLOSE_POS);
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
    if (!clawStates.values()[p_state.ordinal()].state&&time - lastTime > FLIP_TIME) {
      if (p_state == clawTargetStates.CLOSE) {
        if (super.getPosition() != CLOSE_POS) {
          super.setPosition(CLOSE_POS);
          LOGGER.log(RFLogger.Severity.INFO, "CLOSING claw");
          lastTime = time;
        }
        clawTargetStates.CLOSE.setStateTrue();
      } else if (p_state == clawTargetStates.GRAB) {
        if (super.getPosition() != GRAB_POS) {
          super.setPosition(GRAB_POS);
          LOGGER.log("GRABBING claw");
          lastTime = time;
        }
        clawTargetStates.GRAB.setStateTrue();
      }
    }
    }

  public void update() {
    for (var i : clawStates.values()) {
      if (super.getPosition() == i.pos && time > lastTime + FLIP_TIME) i.setStateTrue(); clawTargetStates.values()[i.ordinal()].state=false;
      if(i.state) packet.put("clawPos", i.name());

    }
    for (var i : clawTargetStates.values()) {
      if (i.state && super.getPosition() != i.pos) flipTo(i);
      if(i.state) packet.put("clawPos", i.name());
    }
  }
}
