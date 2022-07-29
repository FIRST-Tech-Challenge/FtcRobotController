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
import org.firstinspires.ftc.teamcode.util.LateInitVal

/**
 * Logically grouped components of the bot's ring shooter.
 *
 * @property indexer The servo that controls throughput of the rings into the flywheel
 * @property motor The motor that powers the shooter.
 *
 * @see [initializedShooter]
 *
 * @author KG
 */
class Shooter {
    var indexer: Servo by LateInitVal()
    var motor: DcMotorEx by LateInitVal()

    fun setIndexerToggled(value: Boolean) {
        indexer.position = if (value) INDEXER_FORWARD else INDEXER_BACK
    }

    fun logIndexerData(telemetry: Telemetry, dataSupplier: DataSupplier<Servo>) {
        telemetry.addData("Indexer", dataSupplier(indexer))
    }

    fun logMotorData(telemetry: Telemetry, dataSupplier: DataSupplier<DcMotorEx>) {
        telemetry.addData("Motor", dataSupplier(motor))
    }

    /**
     * The constants that represent the servo positions for the indexer.
     *
     * @property INDEXER_FORWARD The position where the indexer pushes a ring into the flywheel.
     * @property INDEXER_BACK The position where the indexer is at rest, not doing anything.
     */
    companion object {
        const val INDEXER_BACK = .51
        const val INDEXER_FORWARD = .60
    }
}

/**
 * Initializes a [Shooter]'s `motor` and `indexer` with their default configurations.
 *
 * Kotlin usage example:
 * ```
 * val shooter = initializedShooter(hardwareMap)
 * ```
 *
 * Java usage example:
 * ```
 * Shooter shooter = ShooterKt.initializedShooter(hardwareMap);
 * ```
 *
 * @param hwMap The [HardwareMap]
 * @return A [Shooter] object with the `indexer` and `motor` initialized.`
 *
 * @author KG
 */
private object IgnoreMeIExistSoKDocsWillActuallyRenderTheDoc
fun initializedShooter(hwMap: HardwareMap) = Shooter().apply {
    indexer = initializedServo("IND", hwMap, pos = Shooter.INDEXER_BACK)
    motor = initializedMotor("SH", hwMap, zpb = ZPB.FLOAT, runMode = DCMode.RUN_USING_ENCODER)
}