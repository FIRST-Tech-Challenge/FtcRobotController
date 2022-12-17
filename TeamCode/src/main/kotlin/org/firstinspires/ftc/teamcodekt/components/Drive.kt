@file:Suppress("LocalVariableName")

package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcodekt.util.DataSupplier
import org.firstinspires.ftc.teamcodekt.util.invoke
import org.firstinspires.ftc.teamcodekt.util.maxMagnitude
import org.firstinspires.ftc.teamcodekt.util.withDeadzone
import kotlin.math.*

class Drivetrain(hwMap: HardwareMap) {
    private val frontLeft  = hwMap<DcMotorEx>(DeviceNames.DRIVE_FL).apply { direction = Direction.REVERSE }
    private val frontRight = hwMap<DcMotorEx>(DeviceNames.DRIVE_FR)
    private val backLeft   = hwMap<DcMotorEx>(DeviceNames.DRIVE_BL).apply { direction = Direction.REVERSE }
    private val backRight  = hwMap<DcMotorEx>(DeviceNames.DRIVE_BR)

    fun drive(gamepad: Gamepad, powerMulti: Double = 0.0) {
        val (x, y, r) = gamepad.getDriveSticks()

        val theta = atan2(y, x)
        val power = hypot(x, y)

        val xComponent = power * cos(theta - PI / 4)
        val yComponent = power * sin(theta - PI / 4)

        val max = maxMagnitude(xComponent, yComponent)

        val powers = doubleArrayOf(
            power * (xComponent / max) + r,
            power * (yComponent / max) - r,
            power * (yComponent / max) + r,
            power * (xComponent / max) - r,
        )

        if (power + abs(r) > 1) {
            powers.onEach { it / (power + abs(r)) }
        }

        val _powerMulti = if (!gamepad.isAnyJoystickTriggered()) 0.0 else powerMulti

        powers.onEach {
            withDeadzone((it * it * it) * _powerMulti, 0.05)
        }

        withEachMotor {
            this.power = powers[it]
        }
    }

    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<DcMotorEx>) {
        val motorCaptions = arrayOf("Front left", "Front right", "Back left", "Back right")

        withEachMotor {
            telemetry.addData("${motorCaptions[it]} motor", dataSupplier(this))
        }
    }

    private fun withEachMotor(transformation: DcMotorEx.(Int) -> Unit) {
        frontLeft .transformation(0)
        frontRight.transformation(1)
        backLeft  .transformation(2)
        backRight .transformation(3)
    }
}
