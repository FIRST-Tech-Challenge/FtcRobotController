package org.firstinspires.ftc.teamcode.classes.triggers;

import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public abstract class TriggerCommandBase implements TriggerCommand {

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
        return this.getClass().getSimpleName();
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
