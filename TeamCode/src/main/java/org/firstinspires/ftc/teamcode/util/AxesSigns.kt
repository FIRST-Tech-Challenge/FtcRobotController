package org.firstinspires.ftc.teamcode.util

/**
 * IMU axes signs in the order XYZ (after remapping).
 */
enum class AxesSigns {
    PPP,
    PPN,
    PNP,
    PNN,
    NPP,
    NPN,
    NNP,
    NNN;

    companion object {
        fun fromBinaryValue(bVal: Int): AxesSigns {
            return when (val maskedVal = bVal and 0x07) {
                0 -> PPP
                1 -> PPN
                2 -> PNP
                3 -> PNN
                4 -> NPP
                5 -> NPN
                6 -> NNP
                7 -> NNN
                else -> throw java.lang.IllegalStateException("Unexpected value for maskedVal: $maskedVal")
            }
        }
    }
}