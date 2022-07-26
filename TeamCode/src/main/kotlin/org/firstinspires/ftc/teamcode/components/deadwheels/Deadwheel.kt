package org.firstinspires.ftc.teamcode.components.deadwheels

import com.qualcomm.robotcore.hardware.DcMotor

@JvmInline
value class Deadwheel(
    private val correspondingMotor: DcMotor,
) {
    val ticks: Int
        get() = correspondingMotor.currentPosition
}
