package org.firstinspires.ftc.teamcode.components.deadwheels

import com.qualcomm.robotcore.hardware.DcMotor

class Deadwheel(
    private val correspondingMotor: DcMotor,
) {
    val ticks
        get() = correspondingMotor.currentPosition

    var prevTicks = 0
        private set

    fun snapshotTicks() {
        prevTicks = correspondingMotor.currentPosition
    }
}
