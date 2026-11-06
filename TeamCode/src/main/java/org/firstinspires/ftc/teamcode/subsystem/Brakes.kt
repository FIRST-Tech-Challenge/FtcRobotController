package org.firstinspires.ftc.teamcode.subsystem

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class Brakes(
    taskManager: TaskManager,
    val follower: Follower,
    val servoLeft: Servo,
    val servoRight: Servo
) : Subsystem() {
    enum class Mode(
        val leftPosition: Double,
        val rightPosition: Double
    ) {
        ON(
            leftPosition = 1.0,
            rightPosition = 0.0
        ),
        OFF(
            leftPosition = 0.0,
            rightPosition = 1.0
        );
        operator fun not() = when (this) {
            ON -> OFF
            OFF -> ON
        }
    }
    var mode: Mode = Mode.OFF
        set(mode) {
            field = mode
            servoLeft.position = mode.leftPosition
            servoRight.position = mode.rightPosition
            if (mode == Mode.ON) follower.mode()
        }

    init {
        off()
        taskManager.loop.action += { preventDriveOnBrake() }
    }

    fun preventDriveOnBrake() {
        if (mode == Mode.ON) follower.stop()
    }

    fun off() {
        mode = Mode.OFF
    }

    fun on() {
        mode = Mode.ON
    }

    fun toggle() {
        mode = !mode
    }

    override fun printState(telemetry: Telemetry) {
        TODO("Not yet implemented")
    }
}
