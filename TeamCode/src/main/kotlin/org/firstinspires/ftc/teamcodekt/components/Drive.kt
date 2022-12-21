@file:Suppress("LocalVariableName")

package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcodekt.util.DataSupplier
import ftc.rouge.blacksmith.util.kt.invoke
import kotlin.math.*

class Drivetrain(hwMap: HardwareMap) {
    private val frontLeft  = hwMap<DcMotorEx>(DeviceNames.DRIVE_FL).apply {
        direction = Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    private val frontRight = hwMap<DcMotorEx>(DeviceNames.DRIVE_FR).apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    private val backLeft   = hwMap<DcMotorEx>(DeviceNames.DRIVE_BL).apply {
        direction = Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    private val backRight  = hwMap<DcMotorEx>(DeviceNames.DRIVE_BR).apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun drive(gamepad: Gamepad, _powerMulti: Double = 0.0) {
//        val (x, y, r) = gamepad.getDriveSticks()
//
//        val theta = atan2(y, x)
//        val power = hypot(x, y)
//
//        val xComponent = power * cos(theta - PI / 4)
//        val yComponent = power * sin(theta - PI / 4)
//
//        val max = maxMagnitude<Double>(xComponent, yComponent)
//
//        val powers = doubleArrayOf(
//            power * (xComponent / max).zeroIfNaN() + r,
//            power * (yComponent / max).zeroIfNaN() - r,
//            power * (yComponent / max).zeroIfNaN() + r,
//            power * (xComponent / max).zeroIfNaN() - r,
//        )
//
//        if (power + abs(r) > 1) {
//            powers.onEach { it / (power + abs(r)) }
//        }
//
//        val _powerMulti = if (!gamepad.isAnyJoystickTriggered()) 0.0 else powerMulti
//
//        powers.onEach {
//            (it * it * it * _powerMulti).withDeadzone(0.05)
//        }
//
//        withEachMotor {
//            this.power = powers[it]
//        }

        val (strafe, speed, rotation) = gamepad.getDriveSticks()

        val direction = atan2(speed, strafe)
        val power = sqrt(speed * speed + strafe * strafe)

        val xComponent = sin(direction - PI / 4) * power
        val yComponent = cos(direction - PI / 4) * power

        val max = max(abs(xComponent), abs(yComponent))

        val flp = ((yComponent / max).takeUnless(kotlin.Double::isNaN) ?: .0) + rotation
        val frp = ((xComponent / max).takeUnless(kotlin.Double::isNaN) ?: .0) - rotation
        val blp = ((xComponent / max).takeUnless(kotlin.Double::isNaN) ?: .0) + rotation
        val brp = ((yComponent / max).takeUnless(kotlin.Double::isNaN) ?: .0) - rotation

        val powerScale = listOf(flp, frp, blp, brp, 1.0).maxOf { abs(it) }

        val powerMulti = if (!gamepad.isAnyJoystickTriggered()) 0.0 else _powerMulti

        setPowers(flp, frp, blp, brp)
        transformPowers { it * powerMulti / powerScale }
    }

    fun setPowers(flp: Number, frp: Number, blp: Number, brp: Number) {
        frontLeft.power = flp.toDouble()
        frontRight.power = frp.toDouble()
        backLeft.power = blp.toDouble()
        backRight.power = brp.toDouble()
    }

    fun transformPowers(scaleFunction: (Double) -> Double) {
        frontLeft.power = scaleFunction(frontLeft.power)
        frontRight.power = scaleFunction(frontRight.power)
        backLeft.power = scaleFunction(backLeft.power)
        backRight.power = scaleFunction(backRight.power)
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
