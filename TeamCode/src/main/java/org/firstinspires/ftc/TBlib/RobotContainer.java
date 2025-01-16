// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.TBlib;

import com.arcrobotics.ftclib.command.button.Trigger;
import java.util.function.BooleanSupplier;



/**
 * This class is the layout for the different robotContainer classes for different robot operation modes 
 * where the bulk of the robot should be declared. 
 * in the implementations of this class there should be the declerations 
 * of all the robot's subsystems and logic componnent (e.g state machines)
 */
public abstract class RobotContainer {
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize all subsystems
    initSubsystems();
    // Configure the trigger bindings
    configureBindings();
    // Initialize the state machine
    initStateMachine();
  }

  /**
   * Use this method to initialize all robot logic components.
   */
  protected void initStateMachine() {
  }

  ;

  /**
   * Use this method to initialize all of the robot's subsystems.
   */
  protected abstract void initSubsystems();

  /**
   * Use this method to define your input to command mappings usually using the {@link Trigger} class.
   * Triggers can be created via the {@link Trigger#Trigger(BooleanSupplier)} constructor
   * or by using classes that extend the (e.g the {@link org.firstinspires.ftc.teamcode.utils.BT.BTCommand} class)
   */
  protected void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /**
   * Adds a deadband to a value to make josticks less sensitive and make joystick drift unnoticeable.
   *
   * @param value    the input value to add deadband to.
   * @param deadband the deadband range.
   * @return 0 if the given value is inside the deadband range, if not returns the input value.
   */
  protected static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Adds a deadband and modifies a given input value to a given power to make small adjustments using joysticks easier.
   *
   * @param value    the input value to modify.
   * @param deadband the deadband range.
   * @param power    the power to take the value to.
   * @return 0 if the given value is inside the deadband range, if not returns the modified input value.
   */
  protected static double modifyAxis(double value, double deadband, int power) {
    // Deadband
    value = deadband(value, deadband);

    // Power the axis
    value = Math.copySign(Math.pow(value, power), value);

    return value;
  }

}