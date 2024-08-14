package org.rustlib.core;

import org.rustlib.commandsystem.Command;

public interface Auton extends OpModeCore {
    default Command getAutonomousCommand() {
        return null;
    }
}
