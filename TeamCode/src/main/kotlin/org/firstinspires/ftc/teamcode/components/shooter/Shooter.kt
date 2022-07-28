package org.firstinspires.ftc.teamcode.components.shooter

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.components.motors.DCMode
import org.firstinspires.ftc.teamcode.components.motors.ZPB
import org.firstinspires.ftc.teamcode.components.motors.initializedMotor
import org.firstinspires.ftc.teamcode.components.servos.initializedServo
import org.firstinspires.ftc.teamcode.util.initializableOnce

class Shooter {
    var indexer: Servo by initializableOnce()
    var motor: DcMotorEx by initializableOnce()

    var power
        get() = motor.power
        set(value) {
            motor.power = if (value > 0.6) value else 0.0
        }

    var indexerToggled
        get() = (indexer.position == INDEXER_FORWARD)
        set(value) {
            indexer.position = if (value) INDEXER_FORWARD else INDEXER_BACK
        }

    fun logIndexerData(telemetry: Telemetry, dataSupplier: (Servo) -> Any) {
        telemetry.addData("Indexer", dataSupplier(indexer))
    }

    fun logMotorData(telemetry: Telemetry, dataSupplier: (DcMotorEx) -> Any) {
        telemetry.addData("Motor", dataSupplier(motor))
    }

    companion object {
        const val INDEXER_BACK = .51
        const val INDEXER_FORWARD = .60
    }
}

fun initializedShooter(hwMap: HardwareMap) = Shooter().apply {
    indexer = initializedServo("IND", hwMap, pos = .5)
    motor = initializedMotor("SH", hwMap, zpb = ZPB.FLOAT, runMode = DCMode.RUN_USING_ENCODER)
}