package org.firstinspires.ftc.teamcode.mmooover

/**
 * Combines robot tracking data from EncoderTracking and
 * @see EncoderTracking
 */
class PoseFromToProcessor(origin: Pose) {
    companion object {
        /**
         * How much to factor in the new value into the running average.
         * The average gets (1 - WEIGHT_NEXT) * old + WEIGHT_NEXT * new
         */
        const val WEIGHT_NEXT = .2
    }

    var last: Pose = origin
    var diff: Pose = Pose.ORIGIN

    var alpha = 0.0

    fun update(frontLeft: Double, frontRight: Double, backLeft: Double, backRight: Double, nextPose: Pose) {
        diff = Pose(
            nextPose.x - last.x,
            nextPose.y - last.y,
            wrapAngle(nextPose.heading - last.heading)
        )
        last = nextPose
        val beta = (frontRight + backRight) - (frontLeft + backLeft)
        val next = diff.heading / beta
        alpha = (1 - WEIGHT_NEXT) * alpha + WEIGHT_NEXT * next
    }
}