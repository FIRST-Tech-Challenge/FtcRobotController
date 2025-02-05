package org.firstinspires.ftc.teamcode.test.mocks

import org.firstinspires.ftc.teamcode.hardware.Encoder

/**
 * Fake encoder that always reads 0.
 */
class NotRealEncoder: Encoder {
    private var isFlippedBacking = false
    override var isFlipped: Boolean
        get() = isFlippedBacking
        set(value) {isFlippedBacking = value}

    override fun getCurrentPosition(): Int = 0
    override fun reset() {}
}