package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.helpers.MotorDirections.Companion.DEFAULT_DIRECTIONS


typealias LeftFrontMotor = DcMotorEx
typealias RightFrontMotor = DcMotorEx
typealias LeftBackMotor = DcMotorEx
typealias RightBackMotor = DcMotorEx



data class MotorDirections(val leftFrontMotor: Direction, val rightFrontMotor: Direction, val leftBackMotor: Direction, val rightBackMotor: Direction) {
    companion object {
        val DEFAULT_DIRECTIONS = MotorDirections(
            leftFrontMotor = FORWARD,
            rightFrontMotor = REVERSE,
            leftBackMotor = REVERSE,
            rightBackMotor = FORWARD
        )
    }
}

@Config
class MecanumDrive(
    private val leftFrontMotor: LeftFrontMotor,
    private val rightFrontMotor: RightFrontMotor,
    private val leftBackMotor: LeftBackMotor,
    private val rightBackMotor: RightBackMotor,
    var telemetry: Telemetry? = null,
    var maximumRpm: Int = 250,
    zeroPowerBehavior: ZeroPowerBehavior = BRAKE,
    encoderMode: RunMode = RUN_USING_ENCODER,
    motorDirections: MotorDirections = DEFAULT_DIRECTIONS
    ) {
    var powerVector = PowerVector(0.0, 0.0, 0.0)
        set(value) {
            field = value
            driveVector(field)
        }
    var zeroPowerBehavior: ZeroPowerBehavior = BRAKE
        set(value) {
            leftFrontMotor.zeroPowerBehavior = value
            rightFrontMotor.zeroPowerBehavior = value
            leftBackMotor.zeroPowerBehavior = value
            rightBackMotor.zeroPowerBehavior = value
            field = value
        }
    var encoderMode: RunMode = RUN_USING_ENCODER
        set(value) {
            leftFrontMotor.mode = value
            rightFrontMotor.mode = value
            leftBackMotor.mode = value
            rightBackMotor.mode = value
            field = value
        }
    var motorDirections: MotorDirections = DEFAULT_DIRECTIONS
        set(value) {
            leftFrontMotor.direction = value.leftFrontMotor
            leftBackMotor.direction = value.rightFrontMotor
            rightFrontMotor.direction = value.leftBackMotor
            rightBackMotor.direction = value.rightBackMotor
            field = value
        }

    init {
        this.zeroPowerBehavior = zeroPowerBehavior
        this.encoderMode = encoderMode
        this.motorDirections = motorDirections
        this.zeroEncoders()
    }

    companion object {
        @JvmField var velocityMultiplier = 10
    }

    fun stop() {
        powerVector = PowerVector(0.0, 0.0, 0.0)
    }

    fun zeroEncoders() {
        leftFrontMotor.mode = STOP_AND_RESET_ENCODER
        rightFrontMotor.mode = STOP_AND_RESET_ENCODER
        leftBackMotor.mode = STOP_AND_RESET_ENCODER
        leftBackMotor.mode = STOP_AND_RESET_ENCODER
    }

    private fun driveVector(vector: PowerVector) {
        val leftFrontVelocity = vector.leftFrontPower() * velocityMultiplier
        val rightFrontVelocity = vector.rightFrontPower() * velocityMultiplier
        val leftBackVelocity = vector.leftBackPower() * velocityMultiplier
        val rightBackVelocity = vector.rightBackPower() * velocityMultiplier

        telemetry?.addData("OmniDrive left front: ", leftFrontVelocity)
        telemetry?.addData("OmniDrive right front: ", rightFrontVelocity)
        telemetry?.addData("OmniDrive left back: ", leftBackVelocity)
        telemetry?.addData("OmniDrive right back: ", rightBackVelocity)

        leftFrontMotor.setVelocity(leftFrontVelocity, AngleUnit.RADIANS)
        rightFrontMotor.setVelocity(rightFrontVelocity, AngleUnit.RADIANS)
        leftBackMotor.setVelocity(leftBackVelocity, AngleUnit.RADIANS)
        rightBackMotor.setVelocity(rightBackVelocity, AngleUnit.RADIANS)
    }
}


