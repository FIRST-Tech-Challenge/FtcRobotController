package org.rustlib.core;

import org.rustlib.commandsystem.Command;

public interface AutonomousCore extends OpModeCore {
    default Command getAutonomousCommand() {
        return null;
    }
}
