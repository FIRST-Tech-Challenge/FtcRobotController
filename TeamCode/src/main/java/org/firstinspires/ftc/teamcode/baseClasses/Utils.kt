package org.firstinspires.ftc.teamcode.baseClasses

import com.qualcomm.robotcore.hardware.Gamepad

val Gamepad.right_trigger_pressed: Boolean
    get() = right_trigger > 0.2f

val Gamepad.left_trigger_pressed: Boolean
    get() = left_trigger > 0.2f

typealias LoggerProvider = (Logger) -> Unit
typealias LoggerFunction = (LoggerProvider) -> Unit