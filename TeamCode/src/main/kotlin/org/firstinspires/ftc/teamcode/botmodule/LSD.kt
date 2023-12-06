package org.firstinspires.ftc.teamcode.botmodule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm
import com.qualcomm.robotcore.hardware.PIDFCoefficients

/**
 * Linear Slide Driver
 */
class LSD(opMode: OpMode, private val slide: DcMotorEx) : BotModule(opMode) {

    private val coefficients = PIDFCoefficients(
        0.0,
        0.0,
        0.0,
        0.0,
        MotorControlAlgorithm.PIDF
    )

    // Enum for the position of the slide
    enum class SlideHeight(val height: Double) {
        UNKNOWN(0),
        PRECISE(1),
        BOTTOM(270),  // bottom, stop here
        LOW(-1350),
        MIDDLE(-2133),
//        HIGH(-2133),
        TOP(-2375);

        constructor(stopPower: Int): this(stopPower.toDouble())
    }

    init {
        slide.targetPosition = 0
        slide.mode = RUN_TO_POSITION
        slide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slide.power = 0.0
        slide.power = 1.0
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

    fun setRow(row: Int): Unit {
        useManual = false
        TODO("automatically set the slide to be aligned with a row (0 to 10, -1 for retracted)")
    }
}