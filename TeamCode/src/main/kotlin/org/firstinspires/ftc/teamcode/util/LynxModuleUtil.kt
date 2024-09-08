package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.system.Misc

/**
 * Collection of utilities for interacting with Lynx modules.
 */
object LynxModuleUtil {
    private val MIN_VERSION = LynxFirmwareVersion(1, 8, 2)

    /**
     * Retrieve and parse Lynx module firmware version.
     * @param module Lynx module
     * @return parsed firmware version
     */
    private fun getFirmwareVersion(module: LynxModule): LynxFirmwareVersion? {
        val versionString: String = module.getNullableFirmwareVersionString() ?: return null

        val parts: Array<String> = versionString.split("[ :,]+".toRegex()).dropLastWhile { it.isEmpty() }
            .toTypedArray()
        return try {
            // note: for now, we ignore the hardware entry
            LynxFirmwareVersion(
                parts[3].toInt(),
                parts[5].toInt(),
                parts[7].toInt()
            )
        } catch (e: java.lang.NumberFormatException) {
            null
        }
    }

    /**
     * Ensure all of the Lynx modules attached to the robot satisfy the minimum requirement.
     * @param hardwareMap hardware map containing Lynx modules
     */
    fun ensureMinimumFirmwareVersion(hardwareMap: HardwareMap) {
        val outdatedModules: MutableMap<String, LynxFirmwareVersion?> = HashMap()
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            val version = getFirmwareVersion(module)
            if (version == null || version < MIN_VERSION) {
                for (name in hardwareMap.getNamesOf(module)) {
                    outdatedModules[name] = version
                }
            }
        }
        if (outdatedModules.isNotEmpty()) {
            val msgBuilder: java.lang.StringBuilder = java.lang.StringBuilder()
            msgBuilder.append("One or more of the attached Lynx modules has outdated firmware\n")
            msgBuilder.append(
                Misc.formatInvariant(
                    "Mandatory minimum firmware version for Road Runner: %s\n",
                    MIN_VERSION.toString()
                )
            )
            for ((key, value) in outdatedModules) {
                msgBuilder.append(
                    Misc.formatInvariant(
                        "\t%s: %s\n", key,
                        value?.toString() ?: "Unknown"
                    )
                )
            }
            throw LynxFirmwareVersionException(msgBuilder.toString())
        }
    }

    /**
     * Parsed representation of a Lynx module firmware version.
     */
    class LynxFirmwareVersion(val major: Int, val minor: Int, val eng: Int) : Comparable<LynxFirmwareVersion> {
        override fun equals(other: Any?): Boolean {
            return if (other is LynxFirmwareVersion) {
                major == other.major && minor == other.minor && eng == other.eng
            } else {
                false
            }
        }

        override fun compareTo(other: LynxFirmwareVersion): Int {
            val majorComp: Int = major.compareTo(other.major)
            if (majorComp == 0) {
                val minorComp: Int = minor.compareTo(other.minor)
                return if (minorComp == 0) {
                    eng.compareTo(other.eng)
                } else {
                    minorComp
                }
            } else {
                return majorComp
            }
        }

        override fun toString(): String {
            return Misc.formatInvariant("%d.%d.%d", major, minor, eng)
        }

        override fun hashCode(): Int {
            var result = major
            result = 31 * result + minor
            result = 31 * result + eng
            return result
        }
    }

    /**
     * Exception indicating an outdated Lynx firmware version.
     */
    class LynxFirmwareVersionException(detailMessage: String?) : java.lang.RuntimeException(detailMessage)
}