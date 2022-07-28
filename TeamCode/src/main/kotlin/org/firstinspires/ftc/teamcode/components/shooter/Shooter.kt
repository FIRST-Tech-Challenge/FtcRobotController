package org.firstinspires.ftc.teamcode.components.shooter

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.components.motors.DCMode
import org.firstinspires.ftc.teamcode.components.motors.ZPB
import org.firstinspires.ftc.teamcode.components.motors.initializedMotor
import org.firstinspires.ftc.teamcode.components.servos.initializedServo
import org.firstinspires.ftc.teamcode.util.DataSupplier
import org.firstinspires.ftc.teamcode.util.initializableOnce

class Shooter {
    var indexer: Servo by initializableOnce()
    var motor: DcMotorEx by initializableOnce()

    fun setIndexerToggled(value: Boolean) {
        indexer.position = if (value) INDEXER_FORWARD else INDEXER_BACK
    }

    fun logIndexerData(telemetry: Telemetry, dataSupplier: DataSupplier<Servo>) {
        telemetry.addData("Indexer", dataSupplier(indexer))
    }

    fun logMotorData(telemetry: Telemetry, dataSupplier: DataSupplier<DcMotorEx>) {
        telemetry.addData("Motor", dataSupplier(motor))
    }

    companion object {
        const val INDEXER_BACK = .51
        const val INDEXER_FORWARD = .60
    }
}

fun initializedShooter(hwMap: HardwareMap) = Shooter().apply {
    indexer = initializedServo("IND", hwMap, pos = Shooter.INDEXER_BACK)
    motor = initializedMotor("SH", hwMap, zpb = ZPB.FLOAT, runMode = DCMode.RUN_USING_ENCODER)
}