package org.firstinspires.ftc.teamcode.dragonswpilib.drive;

/**
 * Common base class for drive platforms.
 *
 * <p>{@link edu.wpi.first.wpilibj.MotorSafety} is enabled by default.
 */
public abstract class RobotDriveBase {
  public static final double kDefaultDeadband = 0.02;
  public static final double kDefaultMaxOutput = 1.0;

  protected double m_deadband = kDefaultDeadband;
  protected double m_maxOutput = kDefaultMaxOutput;

  /** The location of a motor on the robot for the purpose of driving. */
  public enum MotorType {
    kFrontLeft(0),
    kFrontRight(1),
    kRearLeft(2),
    kRearRight(3),
    kLeft(0),
    kRight(1),
    kBack(2);

    public final int value;

    MotorType(int value) {
      this.value = value;
    }
  }

  /** RobotDriveBase constructor. */
  public RobotDriveBase() {
    //setSafetyEnabled(true);
  }

  /**
   * Sets the deadband applied to the drive inputs (e.g., joystick values).
   *
   * <p>The default value is {@value #kDefaultDeadband}. Inputs smaller than the deadband are set to
   * 0.0 while inputs larger than the deadband are scaled from 0.0 to 1.0. See {@link
   * edu.wpi.first.math.MathUtil#applyDeadband}.
   *
   * @param deadband The deadband to set.
   */
  public void setDeadband(double deadband) {
    m_deadband = deadband;
  }

  /**
   * Configure the scaling factor for using drive methods with motor controllers in a mode other
   * than PercentVbus or to limit the maximum output.
   *
   * <p>The default value is {@value #kDefaultMaxOutput}.
   *
   * @param maxOutput Multiplied with the output percentage computed by the drive functions.
   */
  public void setMaxOutput(double maxOutput) {
    m_maxOutput = maxOutput;
  }

  /**
   * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
   *
   * @param wheelSpeeds List of wheel speeds to normalize.
   */
  protected static void normalize(double[] wheelSpeeds) {
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    for (int i = 1; i < wheelSpeeds.length; i++) {
      double temp = Math.abs(wheelSpeeds[i]);
      if (maxMagnitude < temp) {
        maxMagnitude = temp;
      }
    }
    if (maxMagnitude > 1.0) {
      for (int i = 0; i < wheelSpeeds.length; i++) {
        wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
      }
    }
  }
}