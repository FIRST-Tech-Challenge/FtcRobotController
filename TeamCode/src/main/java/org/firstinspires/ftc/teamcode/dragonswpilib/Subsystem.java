package org.firstinspires.ftc.teamcode.dragonswpilib;

public interface Subsystem {
    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    default void periodic() {}

    /**
     * Sets the default {@link Command} of the subsystem. The default command will be automatically
     * scheduled when no other commands are scheduled that require the subsystem. Default commands
     * should generally not end on their own, i.e. their {@link Command#isFinished()} method should
     * always return false. Will automatically register this subsystem with the {@link
     * CommandScheduler}.
     *
     * @param defaultCommand the default command to associate with this subsystem
     */
    default void setDefaultCommand(Command defaultCommand) {
        CommandScheduler.getInstance().setDefaultCommand(this, defaultCommand);
    }
}
