package org.firstinspires.ftc.teamcode.baseClasses

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix

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
     *     log.text("Hello, World!")
     *     log.text("Label :: Value")
     *     log.location(OpenGLMatrix())
     * }
     * ```
     */
    fun logger(actions: (Logger) -> Unit) {
        val logger = Logger()
        actions(logger)
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
}