package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

/** Harry Class to contain all Arm functions */
public class Arm extends RFServo {
  static double LOWER_LIMIT = 0.28;
  static double UPPER_LIMIT = 0.98;
  private double lastTime = 0, FLIP_TIME = 0.8;

  /** constructs arm servo, logs to general with CONFIG severity */
  public Arm() {
    super("armServo", 1.0);
    super.setPosition(LOWER_LIMIT);
    super.setFlipTime(FLIP_TIME);
    lastTime = -100;
    super.setLastTime(-100);
    ArmTargetStates.UNFLIPPED.setStateTrue();
  }

  /** enum for arm servo states, built in function to update states */
  public enum ArmStates {
    UNFLIPPED(true),
    FLIPPED(false);

    boolean state = false;

    ArmStates(boolean p_state) {
      state = p_state;
    }

    void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < Arm.ArmStates.values().length; i++) {
          if (Arm.ArmStates.values()[i].state) {
            LOGGER.log("assigned false to position state: " + (Arm.ArmStates.values()[i].name()));
          }
          Arm.ArmStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log("assigned true to position state: " + this.name());
      }
    }

    public boolean getState() {
      return this.state;
    }
  }

  public enum ArmTargetStates {
    UNFLIPPED(true, LOWER_LIMIT),
    FLIPPED(false, UPPER_LIMIT);

    boolean state = false;
    double position;

    ArmTargetStates(boolean p_state, double p_position) {
      state = p_state;
      p_position = position;
    }

    void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < Arm.ArmTargetStates.values().length; i++) {
          if (Arm.ArmTargetStates.values()[i].state) {
            LOGGER.log(
                "assigned false to target state: " + (Arm.ArmTargetStates.values()[i].name()));
          }
          Arm.ArmTargetStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log("assigned true to target state: " + this.name());
      }
    }

    boolean getState() {
      return this.state;
    }
  }

  /**
   * void, toggles arm between outtake position and intake position logs to general with highest
   * verbosity
   */
  public void flip() {
    if (!(Lift.LiftMovingStates.AT_ZERO.state || Lift.LiftPositionStates.AT_ZERO.state)) {
      if (!ArmTargetStates.UNFLIPPED.state) {
        super.setPosition(LOWER_LIMIT);
        LOGGER.log(RFLogger.Severity.INFO, "flipping up");
        ArmTargetStates.UNFLIPPED.setStateTrue();
        Lift.LiftMovingStates.LOW.setStateTrue();
        lastTime = time;
      } else if (!ArmTargetStates.FLIPPED.state) {
        super.setPosition(LOWER_LIMIT);
        LOGGER.log(RFLogger.Severity.INFO, "flipping down");
        ArmTargetStates.FLIPPED.setStateTrue();
        Lift.LiftMovingStates.LOW.setStateTrue();
        lastTime = time;
      }
    } else {
      if (ArmTargetStates.UNFLIPPED.getState()) {
        ArmTargetStates.FLIPPED.setStateTrue();
        Lift.LiftMovingStates.LOW.setStateTrue();
      } else {
        ArmTargetStates.UNFLIPPED.setStateTrue();
        Lift.LiftMovingStates.LOW.setStateTrue();
      }
      LOGGER.log("LIFT AT DANGER ZONE, can't flip!");
    }
  }

  public void flipTo(ArmStates p_state) {
    if (!p_state.state && time - lastTime > FLIP_TIME) {
      if (!(Lift.LiftMovingStates.AT_ZERO.state || Lift.LiftPositionStates.AT_ZERO.state)
          && !Wrist.WristStates.FLAT.state) {
        if (p_state == ArmStates.UNFLIPPED) {
          super.setPosition(LOWER_LIMIT);
          if (Wrist.WristTargetStates.DROP.state || Wrist.WristTargetStates.FLAT.state)
            Wrist.WristTargetStates.FLIP.setStateTrue();
          LOGGER.log(RFLogger.Severity.INFO, "flipping down");
          ArmTargetStates.UNFLIPPED.setStateTrue();
          Lift.LiftMovingStates.LOW.setStateTrue();
          lastTime = time;
        } else if (p_state == ArmStates.FLIPPED) {
          super.setPosition(UPPER_LIMIT);
          if(Wrist.WristTargetStates.HOLD.state||Wrist.WristTargetStates.FLAT.state)Wrist.WristTargetStates.FLIP.setStateTrue();
          LOGGER.log(RFLogger.Severity.INFO, "flipping up");
          ArmTargetStates.FLIPPED.setStateTrue();
          Lift.LiftMovingStates.LOW.setStateTrue();
          lastTime = time;
        } else {
          ArmTargetStates.values()[p_state.ordinal()].setStateTrue();
          LOGGER.log("LIFT AT DANGER ZONE, can't flip!");
        }
      } else {
        LOGGER.log("Lift at zero or wrist flat");
      }
    }
    ArmTargetStates.values()[p_state.ordinal()].setStateTrue();
  }

  public void flipToAuto(ArmStates p_state) {
    if (!Lift.LiftPositionStates.AT_ZERO.state) {
      if (p_state == ArmStates.FLIPPED) {
        super.setPosition(UPPER_LIMIT);
        if (Wrist.WristStates.HOLD.state) Wrist.WristTargetStates.FLIP.setStateTrue();
        LOGGER.log(RFLogger.Severity.INFO, "flipping down");
        ArmTargetStates.FLIPPED.setStateTrue();
        lastTime = time;
      } else {
        super.setPosition(LOWER_LIMIT);
        if(Wrist.WristTargetStates.DROP.state)Wrist.WristTargetStates.FLIP.setStateTrue();
        LOGGER.log(RFLogger.Severity.INFO, "flipping up");
        ArmTargetStates.UNFLIPPED.setStateTrue();
        Lift.LiftMovingStates.LOW.setStateTrue();
        lastTime = time;
      }
    }
    ArmTargetStates.values()[p_state.ordinal()].setStateTrue();
  }

  /**
   * void, sets arm servo position to input position logs to general with highest verbosity
   *
   * @param p_target target position
   */
  public void setPosition(double p_target) {
    super.setPosition(p_target);
  }

  /**
   * void, updates state machine according to where servo is logs to general inside the enum
   * functions
   */
  public void update() {
    if (super.getPosition() == LOWER_LIMIT && time - super.getLastTime() > FLIP_TIME) {
      ArmStates.UNFLIPPED.setStateTrue();
    } else if (super.getPosition() == UPPER_LIMIT && time - super.getLastTime() > FLIP_TIME) {
      ArmStates.FLIPPED.setStateTrue();
    } else {
      LOGGER.log("bruh" + (super.getPosition() == LOWER_LIMIT) + super.getPosition());
    }
    for (var i : ArmTargetStates.values()) {
      if (i.state && super.getTarget() != i.position) {
        flipTo(ArmStates.values()[i.ordinal()]);
      }
    }
  }
}
