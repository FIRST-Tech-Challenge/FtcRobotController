package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo

/**
 * Linear Slide Driver
 */
class LSD(private val opMode: OpMode, private val slide: DcMotorEx) {

    private val coefficients = PIDFCoefficients(
        0.0,
        0.0,
        0.0,
        0.0,
        MotorControlAlgorithm.PIDF
    )

    init {
        slide.mode = RUN_TO_POSITION
        slide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//        slide.targetPositionTolerance = 1
        slide.setPIDFCoefficients(RUN_TO_POSITION, coefficients)
    }

    private var targetHeight: Double = 0.0
    // some constant, test later
    private val maxPosition: Int = 1

    @Suppress("MemberVisibilityCanBePrivate")
    var useManual: Boolean = true

    fun setHeight(pos: Double): Unit {
        useManual = true
        TODO()
//        targetHeight = pos
    }

    fun addHeight(offset: Double): Unit {
        TODO()
    }

    fun setHeight(row: Int): Unit {
        useManual = false
        TODO("automatically set the slide to be aligned with a row (0 to 10, -1 for retracted)")
    }
}