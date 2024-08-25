package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.mmooover.Speed2Power
import org.junit.jupiter.api.Test

class TestSpeedToPower {
    companion object {
        val zeroThresh = Speed2Power(0.0)
        val oneThresh = Speed2Power(1.0)
    }
    @Test
    fun zeroThreshZeroSpeed() {
        FloatUtil.assertClose(zeroThresh.speed2power(0.0), 0.0)
    }
    @Test
    fun zeroThreshFullSpeed() {
        FloatUtil.assertClose(zeroThresh.speed2power(1.0), 1.0)
        FloatUtil.assertClose(zeroThresh.speed2power(-1.0), -1.0)
    }
    @Test
    fun oneThreshAlways1() {
        FloatUtil.assertClose(oneThresh.speed2power(1.0), 1.0)
        FloatUtil.assertClose(oneThresh.speed2power(.1), 1.0)
        FloatUtil.assertClose(oneThresh.speed2power(-1.0), -1.0)
        FloatUtil.assertClose(oneThresh.speed2power(-.1), -1.0)
    }
    @Test
    fun nearZeroAlways0() {
        FloatUtil.assertClose(zeroThresh.speed2power(0.0), 0.0)
        FloatUtil.assertClose(zeroThresh.speed2power(0.000000001), 0.0)
        FloatUtil.assertClose(zeroThresh.speed2power(-0.000000001), 0.0)
        FloatUtil.assertClose(oneThresh.speed2power(0.0), 0.0)
        FloatUtil.assertClose(oneThresh.speed2power(0.000000001), 0.0)
        FloatUtil.assertClose(oneThresh.speed2power(-0.000000001), 0.0)
    }
}