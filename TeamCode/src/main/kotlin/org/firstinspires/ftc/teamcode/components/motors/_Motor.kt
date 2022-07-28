package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util._get

typealias ZPB = DcMotor.ZeroPowerBehavior
typealias DCDirection = DcMotorSimple.Direction
typealias DCMode = DcMotor.RunMode

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
            direction = if (reversed) DCDirection.REVERSE else DCDirection.FORWARD
        }
}