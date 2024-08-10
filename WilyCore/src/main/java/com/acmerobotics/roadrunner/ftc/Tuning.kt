package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.MotorFeedforward
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

enum class DriveType {
    MECANUM,
    TANK
}

class DriveView(
        val type: DriveType,
        val inPerTick: Double,
        val maxVel: Double,
        val minAccel: Double,
        val maxAccel: Double,
        val lynxModules: List<LynxModule>,
        // ordered front to rear
        val leftMotors: List<DcMotorEx>,
        val rightMotors: List<DcMotorEx>,
        // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
        //                  (parEncs.isEmpty() && perpEncs.isEmpty())
        leftEncs: List<Encoder>,
        rightEncs: List<Encoder>,
        parEncs: List<Encoder>,
        perpEncs: List<Encoder>,
        val imu: IMU,
        val voltageSensor: VoltageSensor,
        val feedforwardFactory: FeedforwardFactory,
) {
}

fun interface FeedforwardFactory {
    fun make(): MotorFeedforward
}

interface DriveViewFactory {
    fun make(h: HardwareMap): DriveView
}

class AngularRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}

class ForwardPushTest(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}

class ForwardRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}

class LateralRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}

class LateralPushTest(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}

class ManualFeedforwardTuner(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}

class MecanumMotorDirectionDebugger(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}
