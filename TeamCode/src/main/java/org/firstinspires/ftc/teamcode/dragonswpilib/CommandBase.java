package org.firstinspires.ftc.teamcode.dragonswpilib;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class CommandBase implements Command {

    protected Set<Subsystem> m_requirements = new HashSet<>();

    public final void addRequirements(Subsystem... requirements) {
        Collections.addAll(m_requirements, requirements);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return m_requirements;
    }

}
