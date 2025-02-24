@file:Suppress("NOTHING_TO_INLINE")

package org.firstinspires.ftc.teamcode.limelight

import android.util.Log
import dev.aether.collaborative_multitasking.OneShot
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.TaskGroup
import dev.aether.collaborative_multitasking.TaskTemplate
import dev.aether.collaborative_multitasking.ext.Pause
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.hardware.HClawProxy
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy
import kotlin.math.abs

@Suppress("unused")
private inline operator fun DoubleArray.component6() = get(5)

@Suppress("unused")
private inline operator fun DoubleArray.component7() = get(6)

@Suppress("unused")
private inline operator fun DoubleArray.component8() = get(7)

@Suppress("unused")
private inline operator fun DoubleArray.component9() = get(8)


object LimelightDetectionMode {
    const val RED = 0b0001
    const val YELLOW = 0b0010
    const val BLUE = 0b0100

    fun toArray(bitset: Int): DoubleArray {
        return doubleArrayOf(
            if ((bitset and RED) > 0) 1.0 else 0.0,
            if ((bitset and YELLOW) > 0) 1.0 else 0.0,
            if ((bitset and BLUE) > 0) 1.0 else 0.0,
        )
    }
}

class LimelightSamplePickup constructor(
    scheduler: Scheduler,
    private val hardware: Hardware,
    private val hSlideProxy: HSlideProxy,
    private val hClawProxy: HClawProxy,
    private val enabled: Int,
    private var angle: Double
) : TaskGroup(scheduler) {
    private val depends = setOf(
        hClawProxy.CONTROL_CLAW,
        hClawProxy.CONTROL_FLIP,
    )

    private fun wait(seconds: Number) = Pause(innerScheduler, seconds.toDouble())
    private fun run(lambda: () -> Unit) = OneShot(innerScheduler, lambda)

    private val lightLeft = hardware.lightLeft
    private val lightRight = hardware.lightRight
    private val clawTwist = hardware.clawTwist
    private val clawColor = hardware.clawColor
    private val pickupPosition: Double
    private val twistDelay: Double

    init {
        if (angle < 90) {
            pickupPosition = Hardware.CLAW_TWIST_INIT - (angle * 0.0037)
        } else {
            angle = 180 - angle
            pickupPosition = Hardware.CLAW_TWIST_INIT + (angle * 0.0037)
        }
        twistDelay = (100.0 + (500 / 0.33) * abs(pickupPosition - Hardware.CLAW_TWIST_INIT)) / 1000.0

        extraDeps.addAll(depends)
        this.with {
            it.add(wait(0.200))
                .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                .then(wait(0.400))
                .then(run { clawTwist.position = pickupPosition })
                .then(wait(0.100))
                .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE))
                .then(wait(0.300))
                .then(run { clawTwist.position = Hardware.CLAW_TWIST_INIT })
                .then(wait(twistDelay))
                .then(hClawProxy.aSetFlip(Hardware.FLIP_UP))
                .then(wait(0.100))
                .then(run {
                    val red = clawColor.red()
                    val green = clawColor.green()
                    val blue = clawColor.blue()

                    val detectBlue = blue - green > 100 && blue - red > 100
                    val detectRed = red - blue > 100 && red - green > 100
                    val detectYellow = green - blue > 100 && green - red > 100 && red >= 350
                    if (!(
                        (detectBlue && (enabled and LimelightDetectionMode.BLUE) > 0) ||
                        (detectRed && (enabled and LimelightDetectionMode.RED) > 0) ||
                        detectYellow
                    )) {
                        hClawProxy.setClaw(Hardware.FRONT_OPEN)
                    } else {
                        val flipThird = 0.66
                        it.add(hClawProxy.aSetFlip(flipThird))
                            .then(run { hardware.clawTwist.position = Hardware.CLAW_TWIST_INIT })
                            .then(hSlideProxy.moveIn())
                            .then(hClawProxy.aSetFlip(Hardware.FLIP_UP))
                            .then(run {Log.w("A", "AAAAAA")})
                    }
                })
        }
    }

    override fun invokeOnStart() {
        super.invokeOnStart()
        lightLeft.position = 0.0
        lightRight.position = 0.0
    }
}

class LimelightSearch @JvmOverloads constructor(
    scheduler: Scheduler,
    private val hardware: Hardware,
    private val hSlideProxy: HSlideProxy,
    private val hClawProxy: HClawProxy,
    private var enabled: Int,
    private val telemetry: Telemetry? = null,
) :
    TaskTemplate(scheduler) {
    companion object {
        private var lastEnabledState = -1
    }

    private val limelight = hardware.limelight
    //private val lightRight = hardware.lightRight
    private val lightLeft = hardware.lightLeft
    private val lightRight = hardware.lightRight

    private val depends = setOf(
        hSlideProxy.CONTROL,
        hClawProxy.CONTROL_CLAW,
        hClawProxy.CONTROL_FLIP,
        Hardware.Locks.Signals,
        Hardware.Locks.Limelight
    )

    override fun requirements() = depends

    private var done = false

    private fun data(caption: String, value: Any?) = when {
        telemetry != null -> telemetry.addData(caption, value)
        else -> null
    }

    private fun line(caption: String) = when {
        telemetry != null -> telemetry.addLine(caption)
        else -> null
    }

    override fun invokeOnStart() {
        if (lastEnabledState != enabled) {
            limelight.updatePythonInputs(LimelightDetectionMode.toArray(enabled))
            lastEnabledState = enabled
        }

        limelight.start()
        lightLeft.position = 0.0
        lightRight.position = 0.0
        lightRight.position = 1.0 // white
        hSlideProxy.moveOutSync()
        hClawProxy.setClaw(Hardware.FRONT_OPEN)
    }

    var isRecognized = false; private set
    private var liveAngle = 0.0

    override fun invokeOnTick() {
        val result = limelight.latestResult
        if (result == null) {
            data("Limelight", "Result is null")
            return
        }
        data("LL Pipeline No", limelight.status.pipelineIndex)
        val pyOut = result.pythonOutput
        val (_, xCoord, yCoord, wVal, hVal, _, angle, _, inside) = pyOut
        val ratio = hVal / wVal
        data("LL: angle", angle)
        data("LL: x", xCoord)
        data("LL: y", yCoord)
        data("LL: w", wVal)
        data("LL: h", hVal)
        data("LL: ratio", ratio)

        liveAngle = angle
        isRecognized = inside > 0.5
        lightLeft.position = if (isRecognized) Hardware.LAMP_GREEN else 0.0
        lightRight.position = if (isRecognized) Hardware.LAMP_GREEN else 0.0
    }

    fun proceed(): Boolean {
        if (!isRecognized) return false
        done = true
        scheduler.add(
            LimelightSamplePickup(
                scheduler,
                hardware,
                hSlideProxy,
                hClawProxy,
                enabled,
                liveAngle
            )
        )
        return true
    }

    override fun invokeIsCompleted() = done

    override fun invokeOnFinish() {
        lightRight.position = 0.0
        lightLeft.position = 0.0
        lightRight.position = 0.0
        limelight.stop()
    }
}