package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.hardware.motors.Motor

class MotorGroup(
    private val leader: Motor,
    vararg followers: Motor
) : Motor(), Iterable<Motor> {
    private val group: List<Motor> = listOf(leader, *followers)

    override fun set(output: Double) = group.forEach { it.set(output) }

    override fun get(): Double = leader.get()

    fun getSpeeds(): List<Double> = group.map { it.get() }

    override fun getVelocity(): Double = leader.correctedVelocity

    fun getVelocities(): List<Double> = group.map { it.correctedVelocity }

    override fun setDistancePerPulse(distancePerPulse: Double): Encoder {
        val encoders = group.map { it.setDistancePerPulse(distancePerPulse) }
        return encoders.first()
    }

    fun getPositions(): List<Double> = group.map { it.distance }

    override fun setRunMode(runmode: RunMode?) {
        runmode?.let {
            group.forEach { it.setRunMode(runmode) }
        }
    }

    override fun setZeroPowerBehavior(behavior: ZeroPowerBehavior?) {
        behavior?.let {
            group.forEach { it.setZeroPowerBehavior(behavior) }
        }
    }

    override fun resetEncoder() {
        group.forEach { it.resetEncoder() }
    }

    override fun stopAndResetEncoder() {
        group.forEach { it.stopAndResetEncoder() }
    }

    override fun setPositionCoefficient(kp: Double) {
        group.forEach { it.positionCoefficient = kp }
    }

    override fun atTargetPosition(): Boolean {
        return leader.atTargetPosition()
    }

    override fun setTargetPosition(target: Int) {
        group.forEach { it.setTargetPosition(target) }
    }

    override fun setTargetDistance(target: Double) {
        group.forEach { it.setTargetDistance(target) }
    }

    override fun setPositionTolerance(tolerance: Double) {
        group.forEach { it.setPositionTolerance(tolerance) }
    }

    override fun setVeloCoefficients(kp: Double, ki: Double, kd: Double) {
        group.forEach { it.setVeloCoefficients(kp, ki, kd) }
    }

    override fun setFeedforwardCoefficients(ks: Double, kv: Double) {
        group.forEach { it.setFeedforwardCoefficients(ks, kv) }
    }

    override fun setFeedforwardCoefficients(ks: Double, kv: Double, ka: Double) {
        group.forEach { it.setFeedforwardCoefficients(ks, kv, ka) }
    }

    /**
     * @return true if the motor group is inverted
     */
    override fun getInverted(): Boolean = leader.inverted

    /**
     * Set the motor group to the inverted direction or forward direction.
     * This directly affects the speed rather than the direction.
     *
     * @param isInverted The state of inversion true is inverted.
     */
    override fun setInverted(isInverted: Boolean) {
        group.forEach { it.inverted = isInverted }
    }

    /**
     * Disables all the motor devices.
     */
    override fun disable() {
        group.forEach { it.disable() }
    }

    /**
     * @return a string characterizing the device type
     */
    override fun getDeviceType(): String {
        return "Motor Group"
    }

    /**
     * Stops all motors in the group.
     */
    override fun stopMotor() {
        group.forEach { it.stopMotor() }
    }

    override fun iterator(): Iterator<Motor> = group.iterator()
}