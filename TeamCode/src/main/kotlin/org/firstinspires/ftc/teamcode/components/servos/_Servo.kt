package org.firstinspires.ftc.teamcode.components.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util._get

typealias SDirection = Servo.Direction

/**
 * Initializes the a servo with the given name and position.
 *
 * Kotlin usage examples:
 * ```
 * val servo = initializedServo("indexer", hardwareMap, 0.3)
 * val ovres = initializedServo("rexedni", hardwareMap, 0.7, reversed = true)
 * ```
 *
 * Java usage examples:
 * ```
 * Servo servo = _ServoKt.initializedServo("indexer", hardwareMap, 0.3);
 * Servo ovres = _ServoKt.initializedServo("rexedni", hardwareMap, 0.7, true);
 * ```
 * _Note: Both language examples produce the exact same objects_
 *
 * @param name The name of the servo
 * @param hwMap The [HardwareMap]
 * @param pos The position of the servo
 * @param reversed Whether or not the servo is reversed; defaults to `false`
 * @return A servo initialized with the given options
 *
 * @author KG
 */
@JvmOverloads
fun initializedServo(
    name: String,
    hwMap: HardwareMap,
    pos: Double,
    reversed: Boolean = false
): Servo {
    return hwMap
        ._get<Servo>(name)
        .apply {
            position = pos
            direction = if (reversed) SDirection.REVERSE else SDirection.FORWARD
        }
}