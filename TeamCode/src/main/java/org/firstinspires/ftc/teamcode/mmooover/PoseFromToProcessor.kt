package org.firstinspires.ftc.teamcode.mmooover

import android.annotation.SuppressLint
import android.util.Log
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.random.Random

/**
 * Combines robot tracking data from EncoderTracking and Pose-to-Pose outputs...
 * @see EncoderTracking
 */
class PoseFromToProcessor(origin: Pose) {
    companion object {
        /**
         * How much to factor in the new value into the running average.
         * The average gets (1 - WEIGHT_NEXT) * old + WEIGHT_NEXT * new
         */
        const val WEIGHT_NEXT = .2
        const val LIMIT = 100
    }

    var total = 0

    private var lastTick = System.nanoTime()
    private var last: Pose = origin
    private var diff: Pose = Pose.ORIGIN
    private var deltaHeading: Double = 0.0

    val xi: MutableList<Double> = mutableListOf()
    val yi: MutableList<Double> = mutableListOf()

    private var alpha = 0.0

    fun update(
        powerDiff: Double,
        nextPose: Pose
    ) {
        val now = System.nanoTime()
        val lastTickCache = lastTick
        lastTick = now
        val duration = (now - lastTickCache) / 1e9 // <s>
        diff = Pose(
            nextPose.x - last.x,
            nextPose.y - last.y,
            wrapAngle(nextPose.heading - last.heading)
        ) // <m, m, rad>
        last = nextPose
        val xI = powerDiff * duration
        val yI = diff.heading
        // TODO: this predictor sucks
        deltaHeading = diff.heading

        if (abs(xI) > 1e-3 && abs(yI) > 1e-3) {
            if (xi.size < LIMIT) {
                xi.add(xI)
                yi.add(yI)
            } else {
                val prop = LIMIT.toDouble() / total
                if (Math.random() < prop) {
                    val ind = Random.Default.nextInt(0, LIMIT)
                    xi[ind] = xI
                    yi[ind] = yI
                }
            }
            total++
        }
//        val next = diff.heading / powerDiff
//        alpha = (1 - WEIGHT_NEXT) * alpha + WEIGHT_NEXT * next
    }

    @SuppressLint("DefaultLocale")
    fun dump() {
        Log.i("data", buildString {
            append("x_i: ")
            xi.forEach { append(String.format("%.4f", it)).append(", ") }
            append("\ny_i: ")
            yi.forEach { append(String.format("%.4f", it)).append(", ") }
        })
    }

    fun getMotionToTarget(target: Pose, robot: TriOdoProvider): Motion {
        val dh = wrapAngle(target.heading - last.heading)
        val dx = target.x - last.x
        val dy = target.y - last.y
        val estHeading = last.heading + (.5 * deltaHeading)
        val forward = cos(estHeading) * dx + sin(estHeading) * dy
        val strafe = sin(estHeading) * dx - cos(estHeading) * dy
        return Motion(forward, strafe, dh * robot.forwardOffset)
    }
}