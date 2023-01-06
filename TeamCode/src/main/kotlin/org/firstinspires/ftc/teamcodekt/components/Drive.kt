@file:Suppress("LocalVariableName")

package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import ftc.rogue.blacksmith.util.kt.invoke
import ftc.rogue.blacksmith.util.kt.maxMagnitudeAbs
import ftc.rogue.blacksmith.util.kt.pow
import ftc.rogue.blacksmith.util.kt.withDeadzone
import ftc.rogue.blacksmith.util.withDeadzone
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcodekt.util.DataSupplier
import java.util.*
import kotlin.math.*

class Drivetrain(hwMap: HardwareMap) {
    private val frontLeft  = hwMap<DcMotorEx>(DeviceNames.DRIVE_FL).apply { direction = Direction.REVERSE }
    private val frontRight = hwMap<DcMotorEx>(DeviceNames.DRIVE_FR)
    private val backLeft   = hwMap<DcMotorEx>(DeviceNames.DRIVE_BL).apply { direction = Direction.REVERSE }
    private val backRight  = hwMap<DcMotorEx>(DeviceNames.DRIVE_BR)

    init {
        withEachMotor {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private var lastPowers = DoubleArray(4)

    fun drive(gamepad: Gamepad, powerMulti: Double) {
        val (x, y, r) = gamepad.getDriveSticks()

//        val x = _x.withDeadzone(.03, 1).withDeadzone<Float>(.03, -1)
//        val y = _y.withDeadzone(.03, 1).withDeadzone<Float>(.03, -1)

        val theta = atan2(y, x)
        val power = hypot(x, y)

        val xComponent = power * cos(theta - PI / 4)
        val yComponent = power * sin(theta - PI / 4)

        val max = maxMagnitudeAbs<Double>(xComponent, yComponent, 1e-16)

        val powers = doubleArrayOf(
            power * (xComponent / max) + r,
            power * (yComponent / max) - r,
            power * (yComponent / max) + r,
            power * (xComponent / max) - r,
        )

        if (power + abs(r) > 1) {
            powers.mapInPlace { it / (power + abs(r)) }
        }

        val _powerMulti = if (!gamepad.isAnyJoystickTriggered()) 0.0 else powerMulti

        powers.mapInPlace { ((it pow 3) * _powerMulti).withDeadzone(0.0125) }

        withEachMotor {
            if (abs(powers[it] - lastPowers[it]) > 0.05) { // Caching motor powers may increase loop times?
                this.power = powers[it]
            }
        }

        lastPowers = powers
    }

    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<DcMotorEx>) {
        val motorCaptions = arrayOf("Front left", "Front right", "Back left", "Back right")

        withEachMotor {
            telemetry.addData("${motorCaptions[it]} motor", dataSupplier(this))
        }
    }

    private fun DoubleArray.mapInPlace(transform: (Double) -> Double) = repeat(size) {
        this[it] = transform(this[it])
    }

    private fun withEachMotor(transformation: DcMotorEx.(Int) -> Unit) {
        frontLeft .transformation(0)
        frontRight.transformation(1)
        backLeft  .transformation(2)
        backRight .transformation(3)
    }
}
