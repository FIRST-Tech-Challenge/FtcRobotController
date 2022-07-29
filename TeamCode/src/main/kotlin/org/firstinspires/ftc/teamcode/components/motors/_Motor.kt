package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util._get

typealias ZPB = DcMotor.ZeroPowerBehavior
typealias DCDir = DcMotorSimple.Direction
typealias DCMode = DcMotor.RunMode

/**
 * Initializes the a motor with the given name and other optional properties.
 *
 * Kotlin usage examples:
 * ```
 * val motor = initializedMotor("frontLeft", hardwareMap)
 * val rotom = initializedMotor("frontRight", hardwareMap, reversed = true)
 * ```
 *
 * Java usage examples:
 * ```
 * DcMotorEx motor = _MotorKt.initializedMotor("frontLeft", hardwareMap);
 * DcMotorEx rotom = _MotorKt.initializedMotor(
 *   "frontRight",
 *   hardwareMap,
 *   DcMotor.ZeroPowerBehavior.BRAKE,
 *   true
 * );
 * ```
 * _Note: Both language examples produce the exact same objects_
 *
 * @param name The name of the servo
 * @param hwMap The [HardwareMap]
 * @param pos The position of the servo
 * @param reversed Whether or not the servo is reversed; defaults to false
 * @return A servo initialized with the given options
 *
 * @author KG
 */
@JvmOverloads
fun initializedMotor(
    name: String,
    hwMap: HardwareMap,
    zpb: ZPB = ZPB.BRAKE,
    reversed: Boolean = false,
    runMode: DCMode = DCMode.RUN_WITHOUT_ENCODER
): DcMotorEx {
    return hwMap
        ._get<DcMotorEx>(name)
        .apply {
            mode = runMode
            zeroPowerBehavior = zpb
            direction = if (reversed) DCDir.REVERSE else DCDir.FORWARD
        }
}