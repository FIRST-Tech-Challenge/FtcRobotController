package org.firstinspires.ftc.teamcode.components.deadwheels

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.components.motors.DriveMotors
import org.firstinspires.ftc.teamcode.util.DataSupplier
import org.firstinspires.ftc.teamcode.util.LateInitVal

class Deadwheels {
    var left: Deadwheel by LateInitVal()
    var right: Deadwheel by LateInitVal()
    var back: Deadwheel by LateInitVal()

    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<Deadwheel>) {
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