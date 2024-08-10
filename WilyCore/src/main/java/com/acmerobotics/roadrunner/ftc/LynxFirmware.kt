@file:JvmName("LynxFirmware")

package com.acmerobotics.roadrunner.ftc

import com.qualcomm.robotcore.hardware.HardwareMap

class LynxFirmwareVersion(
    val major: Int,
    val minor: Int,
    val eng: Int
) {
    operator fun compareTo(other: LynxFirmwareVersion): Int {
        val majorComp = major.compareTo(other.major)
        return if (majorComp == 0) {
            val minorComp = minor.compareTo(other.minor)
            if (minorComp == 0) {
                eng.compareTo(other.eng)
            } else {
                minorComp
            }
        } else {
            majorComp
        }
    }

    override fun toString() = "$major.$minor.$eng"
}

fun parse(s: String?): LynxFirmwareVersion? {
    if (s == null) {
        return null
    }

    val parts = s.split("[ :,]+".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
    return try {
        // For now, we ignore the hardware entry
        LynxFirmwareVersion(
            Integer.parseInt(parts[3]),
            Integer.parseInt(parts[5]),
            Integer.parseInt(parts[7])
        )
    } catch (e: NumberFormatException) {
        null
    }
}

private val MIN_VERSION = LynxFirmwareVersion(1, 8, 2)

fun throwIfModulesAreOutdated(hardwareMap: HardwareMap) {
}