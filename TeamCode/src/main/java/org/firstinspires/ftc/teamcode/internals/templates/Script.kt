package org.firstinspires.ftc.teamcode.internals.templates

/**
 * Abstract base class for scripts that can be run in a separate thread.
 */
abstract class Script {
    /**
     * The thread in which the script runs.
     */
    val thread: Thread = Thread { run() }

    /**
     * Checks if the script is still active.
     *
     * @return True if the script is active, false otherwise.
     */
    fun scriptIsActive(): Boolean = !thread.isInterrupted && BaseOpMode.instance?.opModeIsActive() ?: false

    /**
     * Initializes the script. To be implemented by subclasses.
     */
    abstract fun init()

    /**
     * Main logic for the script. To be implemented by subclasses.
     */
    abstract fun run()

    /**
     * Called when the script is stopped. To be implemented by subclasses.
     */
    abstract fun onStop()
}