package org.firstinspires.ftc.teamcode.components.deadwheels

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.components.motors.Motors

fun initializedDeadwheels(motors: Motors) = Deadwheels().apply {
    left = Deadwheel(motors.backRight)
    right = Deadwheel(motors.frontLeft)
    back = Deadwheel(motors.frontRight)
}

fun Deadwheels.logDeadwheelData(telemetry: Telemetry, dataSupplier: (Deadwheel) -> Any) {
    telemetry.addData("Left wheel:", dataSupplier(left))
    telemetry.addData("Right wheel:", dataSupplier(right))
    telemetry.addData("Back wheel:", dataSupplier(back))
}