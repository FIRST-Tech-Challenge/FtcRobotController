package org.firstinspires.ftc.teamcode.components.deadwheels

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.components.motors.DriveMotors
import org.firstinspires.ftc.teamcode.util.initializableOnce

class Deadwheels {
    var left: Deadwheel by initializableOnce()
    var right: Deadwheel by initializableOnce()
    var back: Deadwheel by initializableOnce()

    fun logData(telemetry: Telemetry, dataSupplier: (Deadwheel) -> Any) {
        telemetry.addData("Left wheel:", dataSupplier(left))
        telemetry.addData("Right wheel:", dataSupplier(right))
        telemetry.addData("Back wheel:", dataSupplier(back))
    }

    fun snapshotTicks() = this.run {
        left.snapshotTicks()
        right.snapshotTicks()
        back.snapshotTicks()
    }
}

fun initializedDeadwheels(motors: DriveMotors) = Deadwheels().apply {
    left = Deadwheel(motors.backRight)
    right = Deadwheel(motors.frontLeft)
    back = Deadwheel(motors.frontRight)
}