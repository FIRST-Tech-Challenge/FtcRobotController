package ftc.rogue.blacksmith.util.kalman

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import ftc.rogue.blacksmith.units.AngleUnit.DEGREES
import ftc.rogue.blacksmith.util.toRad

class KalmanTwoWheelLocalizer(
    private val localizer: TwoTrackingWheelLocalizer,
) : TwoTrackingWheelLocalizer(localizer.getWheelPoses()) {

    // Remember heading is in radians
    private val headingFilter = KalmanFilter(0.25, 0.125)
    private val wheelPos1Filter = KalmanFilter(9.0, 11.0)
    private val wheelPos2Filter = KalmanFilter(9.0, 11.0)
    private val headingVelocityFilter = KalmanFilter(0.500, 0.225)
    private val wheelPos1VelocityFilter = KalmanFilter(8.0, 7.0)
    private val wheelPos2VelocityFilter = KalmanFilter(8.0, 7.0)

    override fun getHeading(): Double {
        return headingFilter.filter(localizer.getHeading())
    }

    override fun getWheelPositions(): List<Double> {
        val poses = localizer.getWheelPositions()

        return listOf(
            wheelPos1Filter.filter(poses[0]),
            wheelPos2Filter.filter(poses[1]),
        )
    }

    override fun getWheelVelocities(): List<Double> {
        val poses = localizer.getWheelVelocities()!!

        return listOf(
            wheelPos1VelocityFilter.filter(poses[0]),
            wheelPos2VelocityFilter.filter(poses[1]),
        )
    }

    override fun getHeadingVelocity(): Double {
        val velocity = localizer.getHeadingVelocity()

        return velocity
            ?.let { headingVelocityFilter.filter(it) }
            ?: 0.0
    }

    fun setHeadingFilterCoeffs(R: Double, Q: Double) = apply {
        headingFilter.setProcessNoise(R)
        headingFilter.setMeasurementNoise(Q)
    }

    fun setWheelPos1FilterCoeffs(R: Double, Q: Double) = apply {
        wheelPos1Filter.setProcessNoise(R)
        wheelPos1Filter.setMeasurementNoise(Q)
    }

    fun setWheelPos2FilterCoeffs(R: Double, Q: Double) = apply {
        wheelPos2Filter.setProcessNoise(R)
        wheelPos2Filter.setMeasurementNoise(Q)
    }

    fun setHeadingVelocityFilterCoeffs(R: Double, Q: Double) = apply {
        headingVelocityFilter.setProcessNoise(R)
        headingVelocityFilter.setMeasurementNoise(Q)
    }

    fun setWheelPos1VelocityFilterCoeffs(R: Double, Q: Double) = apply {
        wheelPos1VelocityFilter.setProcessNoise(R)
        wheelPos1VelocityFilter.setMeasurementNoise(Q)
    }

    fun setWheelPos2VelocityFilterCoeffs(R: Double, Q: Double) = apply {
        wheelPos2VelocityFilter.setProcessNoise(R)
        wheelPos2VelocityFilter.setMeasurementNoise(Q)
    }

    // -- INTERNAL --

    companion object {
        private fun TwoTrackingWheelLocalizer.getWheelPoses(): List<Pose2d> {
            val parallelX = this::class.java.getField("PARALLEL_X").get(null) as Double
            val parallelY = this::class.java.getField("PARALLEL_Y").get(null) as Double
            val perpendicularX = this::class.java.getField("PERPENDICULAR_X").get(null) as Double
            val perpendicularY = this::class.java.getField("PERPENDICULAR_Y").get(null) as Double

            return listOf(
                Pose2d(parallelX, parallelY, 0.0),
                Pose2d(perpendicularX, perpendicularY, 90.toRad(from = DEGREES)),
            )
        }
    }
}
