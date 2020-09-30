package org.firstinspires.ftc.teamcode.baseClasses

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class BaseLinearOpMode: LinearOpMode() {
    lateinit var hardware: RobotHardware

    /**
     * Immediately stop the robot
     */
    fun stopMotion() {
        hardware.base.motors.forEach { motor -> motor.power = 0.0 }
    }


    /**
     * Provides a telemetry logging context. Is passed in a log closure, and any data passed to that closure inside the
     * context of this method will be added to the log queue. The logger will log all the items in the queue once the
     * `actions` lambda returns
     *
     * ```kotlin
     * logger { log ->
     *     log("Hello, World!")
     *     log("Label :: Value")
     * }
     * ```
     */
    fun logger(actions: (AddLog) -> Unit) {
        val logger = Logger()
        actions(logger::addLog)
        logger.log(telemetry)
    }


    /**
     * The main runloop of the opmode. You must implement this in every subclass of [BaseLinearOpMode]
     *
     * ```kotlin
     * class SomeOpMode: BaseLinearOpMode() {
     *     override fun runLoop() {
     *         // ... This is the main loop of the opmode ...
     *     }
     * }
     */
    @Throws(InterruptedException::class)
    abstract fun runLoop()


    @Throws(InterruptedException::class)
    override fun runOpMode() {
        hardware = RobotHardware(hardwareMap)
        runLoop()
    }


    private class Logger {
        private var logs: MutableList<Pair<String, String>> = mutableListOf()

        fun addLog(log: String) {
            val split = log.split("::")
            if (split.count() == 2) {
                logs.add(Pair(split[0].trim(), split[1].trim()))
            } else {
                logs.add(Pair("", log))
            }
        }

        fun log(telemetry: Telemetry) {
            telemetry.clear()
            logs.forEach { telemetry.addData(it.first, it.second) }
            telemetry.update()
        }
    }
}