package org.firstinspires.ftc.teamcode.components.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util._get

typealias SDirection = Servo.Direction

@JvmOverloads
fun initializedServo(name: String, hwMap: HardwareMap, pos: Double, reversed: Boolean = false): Servo {
    return hwMap
        ._get<Servo>(name)
        .apply {
            position = pos
            direction = if (reversed) SDirection.REVERSE else SDirection.FORWARD
        }
}

fun initializedServos(hardwareMap: HardwareMap) = Servos().apply {
    indexer = initializedServo("IND", hardwareMap, pos = .5)
}

fun Servos.logServoData(telemetry: Telemetry, dataSupplier: (Servo) -> Any) {
    telemetry.addData("IND", dataSupplier(indexer))
}
