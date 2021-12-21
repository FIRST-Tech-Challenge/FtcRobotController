// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.controller;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.commandftc.RobotUniversal;

import edu.wpi.first.wpiutil.math.MathUtil;

/** Implements a PID control loop. */
@SuppressWarnings("PMD.TooManyFields")
public class PIDController implements AutoCloseable {
  private static int instances;

  // Factor for "proportional" control
  private double m_kp;

  // Factor for "integral" control
  private double m_ki;

  // Factor for "derivative" control
  private double m_kd;

  private double m_maximumIntegral = 1.0;

  private double m_minimumIntegral = -1.0;

  private double m_maximumInput;

  private double m_minimumInput;

  // Do the endpoints wrap around? eg. Absolute encoder
  private boolean m_continuous;

  // The error at the time of the most recent call to calculate()
  private double m_positionError;
  private double m_velocityError;

  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  private double m_prevError;

  // The sum of the errors for use in the integral calc
  private double m_totalError;

  // The error that is considered at setpoint.
  private double m_positionTolerance = 0.05;
  private double m_velocityTolerance = Double.POSITIVE_INFINITY;

  private double m_setpoint;
  private double m_measurement;

  private double last_time = 0;

  /**
   * Allocates a PIDController with the given constants for kp, ki, and kd.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   */
  public PIDController(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    instances++;
  }

  public PIDController(PIDCoefficients coefficients) {
    this(coefficients.p, coefficients.i, coefficients.d);
  }

  @Override
  public void close() {

  }

  /**
   * Sets the PID Controller gain parameters.
   *
   * <p>Set the proportional, integral, and differential coefficients.
   *
   * @param kp The proportional coefficient.
   * @param ki The integral coefficient.
   * @param kd The derivative coefficient.
   */
  public void setPID(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
  }

  /**
   * Sets the Proportional coefficient of the PID controller gain.
   *
   * @param kp proportional coefficient
   */
  public void setP(double kp) {
    m_kp = kp;
  }

  /**
   * Sets the Integral coefficient of the PID controller gain.
   *
   * @param ki integral coefficient
   */
  public void setI(double ki) {
    m_ki = ki;
  }

  /**
   * Sets the Differential coefficient of the PID controller gain.
   *
   * @param kd differential coefficient
   */
  public void setD(double kd) {
    m_kd = kd;
  }

  /**
   * Get the Proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP() {
    return m_kp;
  }

  /**
   * Get the Integral coefficient.
   *
   * @return integral coefficient
   */
  public double getI() {
    return m_ki;
  }

  /**
   * Get the Differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD() {
    return m_kd;
  }

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  public double getSetpoint() {
    return m_setpoint;
  }

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atSetpoint() {
    double positionError;
    if (m_continuous) {
      positionError =
          MathUtil.inputModulus(m_setpoint - m_measurement, m_minimumInput, m_maximumInput);
    } else {
      positionError = m_setpoint - m_measurement;
    }

    double velocityError = (positionError - m_prevError) / (RobotUniversal.opMode.getRuntime() - last_time);

    return Math.abs(positionError) < m_positionTolerance
        && Math.abs(velocityError) < m_velocityTolerance;
  }

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  /** Disables continuous input. */
  public void disableContinuousInput() {
    m_continuous = false;
  }

  /** Returns true if continuous input is enabled. */
  public boolean isContinuousInputEnabled() {
    return m_continuous;
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>When the cap is reached, the integrator value is added to the controller output rather than
   * the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_minimumIntegral = minimumIntegral;
    m_maximumIntegral = maximumIntegral;
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    return m_positionError;
  }

  /** Returns the velocity error. */
  public double getVelocityError() {
    return m_velocityError;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   */
  public double calculate(double measurement, double setpoint) {
    // Set setpoint to provided value
    setSetpoint(setpoint);
    return calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  public double calculate(double measurement) {
    m_measurement = measurement;
    m_prevError = m_positionError;
    double period = get_period();

    if (m_continuous) {
      m_positionError =
          MathUtil.inputModulus(m_setpoint - measurement, m_minimumInput, m_maximumInput);
    } else {
      m_positionError = m_setpoint - measurement;
    }

    m_velocityError = (m_positionError - m_prevError) / period;

    if (m_ki != 0) {
      m_totalError =
          MathUtil.clamp(
              m_totalError + m_positionError * period,
              m_minimumIntegral / m_ki,
              m_maximumIntegral / m_ki);
    }

    return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
  }

  /** Resets the previous error and the integral term. */
  public void reset() {
    m_prevError = 0;
    m_totalError = 0;
  }

  private double get_period() {
    double time = RobotUniversal.opMode.getRuntime();
    time -= last_time;
    last_time = time;
    return time;
  }
}
