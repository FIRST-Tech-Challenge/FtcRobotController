package org.firstinspires.ftc.teamcode.internals.templates

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

/**
 * Abstract base class for all operation modes.
 * Extends LinearOpMode to provide a linear flow of execution.
 * Must be annotated with `@TeleOp` or `@Autonomous` to appear on the driver station.
 */
abstract class BaseOpMode : LinearOpMode() {
    /**
     * List of sub-threads for each script.
     */
    private val subThreads: List<Thread>
        get() = scripts.map { it.thread }

    /**
     * List of all threads including the main thread and sub-threads.
     */
    private val allThreads: List<Thread>
        get() = listOf(mainThread) + subThreads

    /**
     * Main thread that runs the operation mode.
     */
    private val mainThread: Thread = Thread { run() }

    /**
     * Runs the operation mode. Initializes hardware, starts threads, and manages script execution.
     */
    override fun runOpMode() {
        gamepad1
        gamepad2
        instance = this
        //Display.reset()
        construct()
        waitForStart()
        started = true
        for (thread in allThreads) {
            thread.start()
        }
        while (opModeIsActive()) {
            for (script in scripts) {
                if (!script.thread.isAlive) {
                    scripts.remove(script)
                }
            }
            //Display.update()
            sleep(50)
        }
        allThreads.forEach { it.interrupt() }
        onStop()
        scripts.forEach { it.onStop() }
    }

    /**
     * Constructs the operation mode. To be implemented by subclasses.
     */
    abstract fun construct()

    /**
     * Main logic for the operation mode. To be implemented by subclasses.
     * Runs only once.
     */
    abstract fun run()

    /**
     * Called when the operation mode is stopped. To be implemented by subclasses.
     */
    abstract fun onStop()

    companion object {
        const val DRIVETRAIN_GROUP_NAME = "Drivetrain"
        const val DEBUG_GROUP_NAME = "Debug"
        const val AUTONOMOUS_GROUP_NAME = "Autonomous"
        const val FULL_GROUP_NAME = "Full"

        /**
         * Adds a script to the list of scripts and starts it if the operation mode has started.
         *
         * @param script The script to add.
         */
        @Synchronized fun addScript(script: Script) {
            script.init()
            if (started) {
                script.thread.start()
            }
            scripts.add(script)
        }

        /**
         * List of scripts to be executed.
         */
        private val scripts: MutableList<Script> = mutableListOf()

        /**
         * Boolean indicating if the operation mode has started.
         */
        private var started = false

        /**
         * The current instance of the operation mode.
         */
        var instance: BaseOpMode? = null
            private set
    }
}