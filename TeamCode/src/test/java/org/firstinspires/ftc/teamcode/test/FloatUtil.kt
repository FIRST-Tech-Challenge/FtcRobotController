package org.firstinspires.ftc.teamcode.test

import kotlin.math.abs

object FloatUtil {
    const val EPS = 1e-6

    @JvmStatic
    @JvmOverloads
    fun assertClose(a: Number, b: Number, lazyMessage: (() -> Any)? = null) {
        val message = lazyMessage ?: {
            String.format("Expected %.4f to be approximately %.4f", a.toDouble(), b.toDouble())
        }
        val diff = abs(a.toDouble() - b.toDouble())
        assert(diff < EPS, message)
    }
}