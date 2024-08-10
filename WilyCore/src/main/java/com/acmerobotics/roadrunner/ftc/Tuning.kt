package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.TankKinematics
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.constantProfile
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.SerialNumber
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min

class MidpointTimer {
    private val beginTs = System.nanoTime()
    private var lastTime: Long = 0

    fun seconds(): Double {
        return 1e-9 * (System.nanoTime() - beginTs)
    }

    fun addSplit(): Double {
        val time = System.nanoTime() - beginTs
        val midTimeSecs = 0.5e-9 * (lastTime + time)
        lastTime = time
        return midTimeSecs
    }
}

private class MutableSignal(
    val times: MutableList<Double> = mutableListOf(),
    val values: MutableList<Double> = mutableListOf()
)

enum class DriveType {
    MECANUM,
    TANK
}

private fun unwrap(e: Encoder): RawEncoder =
    when (e) {
        is OverflowEncoder -> e.encoder
        is RawEncoder -> e
    }

fun interface FeedforwardFactory {
    fun make(): MotorFeedforward
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
    val imu: LazyImu,
    val voltageSensor: VoltageSensor,
    val feedforwardFactory: FeedforwardFactory,
) {
    fun setBulkCachingMode(mode: LynxModule.BulkCachingMode) {
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
    }
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


class DeadWheelDirectionDebugger(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
    }
}