package org.firstinspires.ftc.teamcode.Components;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

/** William */
public class Lift extends RFDualMotor {
  private double lastPower = 0.0;
  private double target = 0.0;
  private double MIN_VELOCITY = 20, MANUAL_TIME = 0.2, lastManualTime = -1.0;
  public static double max = 1500,
      min = -10,
      RESISTANCE = 450,
      kS = 0.03,
      kV = 3.2786E-4,
      kA = 4E-5,
      MAX_UP_VELO = 3000,
      MAX_DOWN_VELO = -2280,
      MAX_ACCEL = 10000,
      MAX_DECEL = -66974,
      kP = 0,
      kD = 0;

  /** Constructor */
  public Lift() {
    super("rightLiftMotor", "leftLiftMotor", !isTeleop);
    super.setDirection(DcMotorSimple.Direction.REVERSE);
    setConstants(
        max, min, RESISTANCE, kS, kV, kA, MAX_UP_VELO, MAX_DOWN_VELO, MAX_ACCEL, MAX_DECEL, kP, kD);
    super.setTarget(0);
    lastPower = 0;
    lastManualTime = -100;
    target = 0;
    if(!isTeleop){
      super.resetPosition();
    }
  }

  /** Stores different states of lift. */
  public enum LiftPositionStates {
    HIGH_SET_LINE(1400, false),
    MID_SET_LINE(800, false),
    LOW_SET_LINE(375, false),
    AT_ZERO(0, true);

    double position;
    boolean state;

    LiftPositionStates(double p_position, boolean p_state) {
      this.position = p_position;
      this.state = p_state;
    }

    void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < LiftPositionStates.values().length; i++) {
          if (LiftPositionStates.values()[i].state) {
            LOGGER.log(
                "assigned false to position state: " + LiftPositionStates.values()[i].name());
          }
          LiftPositionStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log(RFLogger.Severity.INFO, "assigned true to position state: " + this.name());
      }
    }

    public boolean getState() {
      return this.state;
    }

    public double getPosition() {
      return position;
    }
  }

  public enum LiftMovingStates {
    HIGH(false),
    MID(false),
    LOW(false),
    AT_ZERO(true);

    boolean state;

    LiftMovingStates(boolean p_state) {
      this.state = p_state;
    }

    void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < LiftMovingStates.values().length; i++) {
          if (LiftMovingStates.values()[i].state) {
            LOGGER.log("assigned false to target state: " + LiftMovingStates.values()[i].name());
          }
          LiftMovingStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log(RFLogger.Severity.INFO, "assigned true to target state: " + this.name());
      }
    }

    public boolean getState() {
      return this.state;
    }

    public void clearTargets() {
      for (LiftMovingStates i : LiftMovingStates.values()) {
        i.state = false;
      }
    }
  }

  /**
   * Depending on which state the lift is currently in, checks whether the state can be transitioned
   * to the next state, then changes state values. Logs which state(s)' values have been changed and
   * to what. Logs to RFMotor & general logs. Logs to finest level. Updates LiftMovingStates and
   * LiftPositionStates state machines.
   */
  public void update() {
    LOGGER.log(
        RFLogger.Severity.FINEST,
        "currentPos: " + super.getCurrentPosition() + ", currentTarget: " + super.getTarget());
    for (var i : LiftPositionStates.values()) {
      if (abs(super.getCurrentPosition() - i.position) < 20) {
        i.setStateTrue();
      }
    }
    if (super.getCurrentPosition() < 350) LiftPositionStates.AT_ZERO.setStateTrue();

    for (var i : LiftMovingStates.values()) {
      if (i.state
          && abs(super.getTarget() - LiftPositionStates.values()[i.ordinal()].position) > 30) {
        setPosition(LiftPositionStates.values()[i.ordinal()]);
      }
    }
    if (isTeleop) {
      if (time - lastManualTime > MANUAL_TIME) {
        setPosition(super.getTarget(),0);
      } else {
        setTarget(super.getCurrentPosition());
        LiftMovingStates.LOW.clearTargets();
      }
    } else {
      setPosition(super.getTarget());
    }
    LOGGER.log(RFLogger.Severity.FINE, "currentPos: " + super.getCurrentPosition());
    packet.put("liftPos", super.getCurrentPosition());
  }

  /**
   * Sets target position for lift.
   *
   * @param p_target target position for lift to run to Logs what position the target position has
   *     been set to. Logs to RFMotor & general logs. Logs to finest level. Updates LiftMovingStates
   *     state machine.
   */
  public void setPosition(double p_target) {
    if (!Wrist.WristStates.FLAT.state) {
      super.setPosition(p_target, 0);
      if (target != p_target) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("lifting to: " + p_target);
        target = p_target;
      }
    } else {
      super.setRawPower(-0.2);
      LOGGER.log(RFLogger.Severity.SEVERE, "Wrist state FLAT, can't move");
    }
  }

  public void setPosition(LiftPositionStates p_state) {
    if (!Wrist.WristStates.FLAT.state) {
      if (p_state.equals(LiftPositionStates.AT_ZERO)) {
        if (Arm.ArmStates.UNFLIPPED.getState()
            && Arm.ArmTargetStates.UNFLIPPED.getState()
            && Wrist.WristStates.HOLD.state) {
          super.setPosition(p_state.position - 10, 0);
        } else {
          super.setPosition(LiftPositionStates.LOW_SET_LINE.position, 0);
        }
      } else {
        super.setPosition(p_state.position, 0);
      }
    } else {
      LOGGER.log(RFLogger.Severity.SEVERE, "Wrist state FLAT, can't move");
    }
    if (!LiftMovingStates.values()[p_state.ordinal()].state) {
      LiftMovingStates.values()[p_state.ordinal()].setStateTrue();
    }
  }

  /**
   * Manually extend/retract slides
   *
   * @param p_power How fast the user wants to move the slides and in what direction Logs that the
   *     lift is currently being manually extended. Logs to RFMotor & general logs. Logs to finest
   *     level. Updates LiftMovingStates state machine.
   */
  public void manualExtend(double p_power) {
    if (!Wrist.WristStates.FLAT.state) {
      super.setPower(p_power);
      lastManualTime = time;
      if (p_power != lastPower) {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("setting power to: " + p_power);
        lastPower = p_power;
      }
    }
  }

  /**
   * Iterate to the next highest set line lift height. Logs what set line lift target height the
   * lift has been set to run to. Logs to RFMotor & general logs. Logs to finest level. Updates
   * LiftMovingStates state machine.
   */
  public void iterateUp() {
    for (var i : LiftMovingStates.values()) {
      if (i.state && i != LiftMovingStates.HIGH) {
        var targetState = LiftPositionStates.values()[(i.ordinal() - 1) % 4];
        setPosition(targetState);
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("iterated up to state: " + targetState);
        break;
      }
    }
  }

  /**
   * Iterate to the next lowest set line lift height. Logs what set line lift target height the lift
   * has been set to run to. Logs to RFMotor & general logs. Logs to finest level. Updates
   * LiftMovingStates state machine.
   */
  public void iterateDown() {
    for (var i : LiftMovingStates.values()) {
      if (i.state && i != LiftMovingStates.AT_ZERO) {
        var targetState = LiftPositionStates.values()[(i.ordinal() + 1) % 4];
        setPosition(targetState);
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("iterated down to state: " + targetState);
        break;
      }
    }
  }
}
