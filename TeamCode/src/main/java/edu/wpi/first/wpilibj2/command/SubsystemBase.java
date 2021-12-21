// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command;

/**
 * A base for subsystems that handles registration in the constructor, and provides a more intuitive
 * method for setting the default command.
 */
public abstract class SubsystemBase implements Subsystem {
  private String name;

  /** Constructor. */
  public SubsystemBase() {
    name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    CommandScheduler.getInstance().registerSubsystem(this);
  }
}
