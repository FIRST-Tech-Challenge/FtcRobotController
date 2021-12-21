// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

@SuppressWarnings("PMD.AbstractClassWithoutAbstractMethod")
public abstract class CommandBase implements Command {

  protected Set<Subsystem> m_requirements = new HashSet<>();
  private String name;

  protected CommandBase() {
    name = getClass().getName();
  }

  /**
   * Adds the specified requirements to the command.
   *
   * @param requirements the requirements to add
   */
  public final void addRequirements(Subsystem... requirements) {
    m_requirements.addAll(Arrays.asList(requirements));
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return m_requirements;
  }

  @Override
  public String getName() {
    return name;
  }
}
