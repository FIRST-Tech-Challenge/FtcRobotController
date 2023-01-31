@file:Suppress("LocalVariableName")

package org.firstinspires.ftc.teamcodekt.components

import com.arcrobotics.ftclib.hardware.RevIMU
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Gamepad
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.invoke
import ftc.rogue.blacksmith.util.kt.maxMagnitudeAbs
import ftc.rogue.blacksmith.util.kt.pow
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames
import java.util.*
import kotlin.math.*

class Drivetrain {
    private val frontLeft  = hwMap<DcMotorEx>(DeviceNames.DRIVE_FL).apply { direction = Direction.REVERSE }
    private val frontRight = hwMap<DcMotorEx>(DeviceNames.DRIVE_FR)
    private val backLeft   = hwMap<DcMotorEx>(DeviceNames.DRIVE_BL).apply { direction = Direction.REVERSE }
    private val backRight  = hwMap<DcMotorEx>(DeviceNames.DRIVE_BR)

    private val imu = RevIMU(hwMap)

    init {
        imu.init()

        withEachMotor {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    var shouldDriveRC = true

    private val signalRC = "\uD83D\uDFE2".repeat(50)
    private val signalFC = "\uD83D\uDD34".repeat(50)

    fun drive(gamepad: Gamepad, powerMulti: Double) {
        if (shouldDriveRC) {
            driveRC(gamepad, powerMulti)
            BlackOp.mTelemetry.addLine(signalRC)
        } else {
            driveFC(gamepad, powerMulti)
            BlackOp.mTelemetry.addLine(signalFC)
        }
    }

    fun switchMode() {
        shouldDriveRC = !shouldDriveRC
    }

    fun resetIMU() {
        imu.reset()
    }

    private fun driveRC(gamepad: Gamepad, powerMulti: Double) {
        val (x, y, _r) = gamepad.getDriveSticks()
        val r = _r * .9

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

        powers.mapInPlace { (it pow 3) * _powerMulti }

        withEachMotor {
            this.power = powers[it]
        }
    }

    private fun driveFC(gamepad: Gamepad, powerMulti: Double) {
        val (x, _y, rx) = gamepad.getDriveSticks()
        val y = -_y

        val heading = Math.toRadians(-imu.heading + 180)
        val rotX = x * cos(heading) - y * sin(heading)
        val rotY = x * sin(heading) + y * cos(heading)

        val powers = doubleArrayOf(
            rotY + rotX - rx,
            rotY - rotX + rx,
            rotY - rotX - rx,
            rotY + rotX + rx,
        )

//        val max = powers.max()
//        if (max > 1) {
//            powers.mapInPlace { it / max }
//        }

        val _powerMulti = if (!gamepad.isAnyJoystickTriggered()) 0.0 else powerMulti

        powers.mapInPlace { (it pow 3) * _powerMulti }

        withEachMotor {
            this.power = powers[it]
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
