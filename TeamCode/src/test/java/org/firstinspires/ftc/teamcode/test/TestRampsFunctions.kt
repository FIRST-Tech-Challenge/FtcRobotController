package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.mmooover.Ramps
import org.firstinspires.ftc.teamcode.test.FloatUtil.assertClose
import org.junit.jupiter.api.Test
import kotlin.math.min

class TestRampsFunctions {
    @Test
    fun testDefaultOperations() {
        val obj = Ramps(null, null)
        for (limit in 10 downTo 1) {
            val limitReal = limit / 10.0
            assertClose(obj.ease(0.0, 0.0, limitReal), limitReal)
        }
    }

    @Test
    fun testRampUpLinear() {
        val obj = Ramps(
            Ramps.linear(1.0),
            null
        )
        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(progressReal, 0.0, 1.0), progressReal)
        }
        assertClose(obj.ease(2.0, 0.0, 1.0), 1.0)
        assertClose(obj.ease(10.0, 0.0, 1.0), 1.0)
    }

    @Test
    fun testRampDownLinear() {
        val obj = Ramps(
            null,
            Ramps.linear(1.0)
        )
        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(0.0, progressReal, 1.0), progressReal)
        }
        assertClose(obj.ease(0.0, 2.0, 1.0), 1.0)
        assertClose(obj.ease(0.0, 10.0, 1.0), 1.0)
    }

    @Test
    fun testLimitClip() {
        val obj = Ramps(
            Ramps.linear(1.0),
            null,
            Ramps.LimitMode.CLIP
        )
        assertClose(obj.ease(0.5, 0.0, 0.5), 0.5)
        assertClose(obj.ease(1.0, 0.0, 0.5), 0.5)
    }

    @Test
    fun testLimitScale() {
        val obj = Ramps(
            Ramps.linear(1.0),
            null,
            Ramps.LimitMode.SCALE
        )
        assertClose(obj.ease(0.5, 0.0, 0.5), 0.25)
        assertClose(obj.ease(1.0, 0.0, 0.5), 0.5)
        assertClose(obj.ease(2.0, 0.0, 0.5), 0.5)
    }

    @Test
    fun testMultiFunction() {
        val obj = Ramps(
            Ramps.linear(1.0),
            Ramps.linear(1.0),
            Ramps.LimitMode.CLIP
        )

        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(progressReal, 10.0, 1.0), progressReal)
        }

        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(10.0, progressReal, 1.0), progressReal)
        }
    }

    @Test
    fun testMultiFunctionClip() {
        val obj = Ramps(
            Ramps.linear(1.0),
            Ramps.linear(1.0),
            Ramps.LimitMode.CLIP
        )

        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(progressReal, 10.0, .5), min(.5, progressReal))
        }

        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(10.0, progressReal, .5), min(.5, progressReal))
        }
    }

    @Test
    fun testMultiFunctionScale() {
        val obj = Ramps(
            Ramps.linear(1.0),
            Ramps.linear(1.0),
            Ramps.LimitMode.SCALE
        )

        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(progressReal, 10.0, .5), .5 * progressReal)
        }

        for (progress in 0..<10) {
            val progressReal = progress / 10.0
            assertClose(obj.ease(10.0, progressReal, .5), .5 * progressReal)
        }
    }
}