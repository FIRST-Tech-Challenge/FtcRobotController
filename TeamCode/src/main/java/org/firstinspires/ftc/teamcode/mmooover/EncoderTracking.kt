package org.firstinspires.ftc.teamcode.mmooover

import org.apache.commons.math3.stat.regression.SimpleRegression
import org.firstinspires.ftc.teamcode.hardware.Encoder
import kotlin.math.cos
import kotlin.math.sin

class EncoderTracking @JvmOverloads constructor(
    encoderSource: TriOdoProvider,
    origin: Pose = Pose.ORIGIN,
) {
    companion object {
        /**
         * How much to factor in the new value into the running average.
         * The average gets (1 - WEIGHT_NEXT) * old + WEIGHT_NEXT * new
         *
         * TODO: this is unused rn
         */
        const val WEIGHT_NEXT = .2
        const val HIST_SIZE = 10
    }

    private fun tick2inch(ticks: Int): Double {
        return (ticks / ticksPerRotation) * 2 * StrictMath.PI * radiusInches
    }

    private val ticksPerRotation: Double = encoderSource.encoderTicksPerRevolution.toDouble()
    private val radiusInches: Double = encoderSource.encoderWheelRadius
    private val trackWidth: Double =
        encoderSource.getTrackWidth() // distance between two parallel encoders
    private val forwardOffset: Double = encoderSource.getForwardOffset()

    var heading: Double = origin.heading
    var x: Double = origin.x
    var y: Double = origin.y
    val currentPose get() = Pose(x, y, heading)

    var lastLeft: Double
    var lastCenter: Double
    var lastRight: Double
    val leftEncoder: Encoder = encoderSource.getLeftEncoder()
    val centerEncoder: Encoder = encoderSource.getCenterEncoder()
    val rightEncoder: Encoder = encoderSource.getRightEncoder()

    private val poseHistory: ArrayDeque<Pose> = ArrayDeque(HIST_SIZE)
    private val moments: ArrayDeque<Double> = ArrayDeque(HIST_SIZE)
    private var lastTick = System.nanoTime()
    private var lastPose = Pose.ORIGIN
    private var deltaPose: Pose = Pose.ORIGIN
    private var deltaHeading = 0.0

    // TriOdoProvider provides the methods for the getting the encoders
    init {
        lastLeft = tick2inch(leftEncoder.getCurrentPosition())
        lastCenter = tick2inch(centerEncoder.getCurrentPosition())
        lastRight = tick2inch(rightEncoder.getCurrentPosition())
        poseHistory.addLast(currentPose)
        moments.addLast(lastTick / 1e9)
    }

    // Updates the pose
    fun step() {
        val now = System.nanoTime()
        val lastTickT = lastTick
        lastTick = now
        val duration: Double = (now - lastTickT) / 1.0e9

        // Current encoder values
        val currentLeft = tick2inch(leftEncoder.getCurrentPosition())
        val currentCenter = tick2inch(centerEncoder.getCurrentPosition())
        val currentRight = tick2inch(rightEncoder.getCurrentPosition())
        // Change in the encoders' values
        val deltaLeft = currentLeft - lastLeft
        val deltaCenter = currentCenter - lastCenter
        val deltaRight = currentRight - lastRight
        // Change in distances
        val deltaTurn = (deltaRight - deltaLeft) / trackWidth
        val deltaForward = (deltaLeft + deltaRight) / 2.0
        val deltaStrafe = deltaCenter - forwardOffset * deltaTurn
        // Predicts how much our robot turns so we know how much forward and strafe we now need - the heading changes enough in each loop that we need to predict it
        val nextHeading = heading + deltaTurn
        val avgHead = (heading + nextHeading) / 2.0
        val deltaX = deltaForward * cos(avgHead) - deltaStrafe * sin(avgHead)
        val deltaY = deltaForward * sin(avgHead) + deltaStrafe * cos(avgHead)

        x += deltaX
        y += deltaY
        heading = nextHeading

        lastLeft = currentLeft
        lastCenter = currentCenter
        lastRight = currentRight

        deltaPose = Pose(
            x - lastPose.x,
            y - lastPose.y,
            wrapAngle(heading - lastPose.heading)
        )

        /*
        Push the new pose onto the queue
         */
        if (poseHistory.size == HIST_SIZE) {
            poseHistory.removeFirst()
            moments.removeFirst()
        }
        poseHistory.addLast(currentPose)
        moments.addLast(now / 1e9)
        /*
        this relies on the fact that currentPose creates a new Pose instance
        holding x, y, and heading.
         */
        lastPose = currentPose
        deltaHeading = deltaPose.heading
    }

    // Gets current pose
    fun getPose() = currentPose

    fun getMotionToTarget(target: Pose, robot: TriOdoProvider): Motion {
        val dh = wrapAngle(target.heading - lastPose.heading)
        val dx = target.x - lastPose.x
        val dy = target.y - lastPose.y
        val estHeading = lastPose.heading + (.5 * deltaHeading)
        val forward = cos(estHeading) * dx + sin(estHeading) * dy
        val strafe = sin(estHeading) * dx - cos(estHeading) * dy
        return Motion(forward, strafe, dh * robot.forwardOffset)
    }
}
