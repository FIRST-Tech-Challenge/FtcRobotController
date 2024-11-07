package org.firstinspires.ftc.teamcode.test.mocks

import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.mmooover.TriOdoProvider

class DummyRobot : TriOdoProvider {
    private val encoder = NotRealEncoder()

    override fun getLeftEncoder() = encoder

    override fun getRightEncoder() = encoder

    override fun getCenterEncoder() = encoder

    override fun getEncoderTicksPerRevolution() = Hardware.ENC_TICKS_PER_REV

    override fun getEncoderWheelRadius() = Hardware.ENC_WHEEL_RADIUS

    override fun getTrackWidth() = Hardware.TRACK_WIDTH

    override fun getForwardOffset() = Hardware.FORWARD_OFFSET
}