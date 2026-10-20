package org.firstinspires.ftc.teamcode.objects;

/**
 * One mechanism on the robot: the drive, the intake, the launcher.
 *
 * <p>A robot program is a loop. Every subsystem answers four questions:
 * what does it remember (its fields), what does it set up once ({@link #Init()}),
 * what does it read from the gamepad each cycle ({@link #HandleUserInput()}),
 * and what does it do each cycle ({@link #Process()}).
 */
public interface Subsystem {
    /**
     * Set up without moving. Called over and over during INIT until every subsystem
     * returns true, so it is fine to return false while a sensor is still warming up.
     *
     * @return true once this subsystem is ready
     */
    boolean Init();

    /** Read the gamepads. Called every loop in TeleOp, before {@link #Process()}. */
    void HandleUserInput();

    /** Do this subsystem's work for one loop. Called every loop in TeleOp and Auto. */
    void Process();
}
