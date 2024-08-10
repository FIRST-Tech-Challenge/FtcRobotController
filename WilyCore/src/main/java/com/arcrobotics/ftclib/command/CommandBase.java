/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.arcrobotics.ftclib.command;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * A base class for {@link Command}s.
 *
 * @author Jackson
 */
@SuppressWarnings("PMD.AbstractClassWithoutAbstractMethod")
public abstract class CommandBase implements Command {

    protected String m_name = this.getClass().getSimpleName();
    protected String m_subsystem = "Ungrouped";
    protected Set<Subsystem> m_requirements = new HashSet<>();

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

    public String getName() {
        return m_name;
    }

    public void setName(String name) {
        m_name = name;
    }

    public String getSubsystem() {
        return m_subsystem;
    }

    public void setSubsystem(String subsystem) {
        m_subsystem = subsystem;
    }

}