package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Magazine.pixels;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

@Config
public class Claw extends RFServo {
  public static double GRAB_POS = 0.28,
      CLOSE_POS = 0.56,
      FLIP_TIME = 0.3, GRAB2=0.8, CLOSE2 = 0.46;
  private double lastTime = 0;
  private Servo servo2;
  public Claw() {
    super("clawServo", 1.0);
    servo2 = op.hardwareMap.get(Servo.class, "clawServo2");
    lastTime = -100;
    super.setLastTime(-100);
    if (!isTeleop) {
      super.setPosition(GRAB_POS);
      servo2.setPosition(GRAB2);
      super.setFlipTime(FLIP_TIME);
      clawStates.GRAB.setStateTrue();
      clawTargetStates.GRAB.setStateTrue();
      clawTargetStates.GRAB.state = false;
    } else{
      super.setPosition(CLOSE_POS);
      servo2.setPosition(CLOSE2);
      super.setFlipTime(FLIP_TIME);
      clawStates.CLOSE.setStateTrue();
      clawTargetStates.CLOSE.setStateTrue();
      clawTargetStates.CLOSE.state = false;
    }
    lastTime = -100;
    super.setLastTime(-100);
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
    if (time - lastTime > FLIP_TIME) {
      if (p_state == clawTargetStates.CLOSE) {
        if (super.getPosition() != CLOSE_POS) {
          super.setPosition(CLOSE_POS);
          LOGGER.log(RFLogger.Severity.INFO, "CLOSING claw");
          lastTime = time;
          servo2.setPosition(CLOSE2);
        }
        clawTargetStates.CLOSE.setStateTrue();
      } else if (p_state == clawTargetStates.GRAB) {
        if (super.getPosition() != GRAB_POS && !Arm.ArmTargetStates.GRAB.getState()) {
          super.setPosition(GRAB_POS);
          servo2.setPosition(GRAB2);
          LOGGER.log("GRABBING claw");
          lastTime = time;
        }
        clawTargetStates.GRAB.setStateTrue();
      }
    }
    }
  public void moveOne(boolean grab) {
    if (grab) {
      super.setPosition(GRAB_POS);
    }
    else{
      super.setPosition(CLOSE_POS);
    }
  }
  public void moveTwo(boolean grab) {
    if (grab) {
      servo2.setPosition(GRAB2);
    }
    else{
      servo2.setPosition(CLOSE2);
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
