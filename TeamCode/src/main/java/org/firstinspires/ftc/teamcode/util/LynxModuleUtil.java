package org.firstinspires.ftc.teamcode.util;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.HashMap;
import java.util.Map;

/**
 * Collection of utilites for interacting with Lynx modules.
 */
public class LynxModuleUtil {

    private static final LynxFirmwareVersion MIN_VERSION = new LynxFirmwareVersion(1, 8, 2);

    /**
     * Parsed representation of a Lynx module firmware version.
     */
    public static class LynxFirmwareVersion implements Comparable {
        public final int major;
        public final int minor;
        public final int eng;

        public LynxFirmwareVersion(int major, int minor, int eng) {
            this.major = major;
            this.minor = minor;
            this.eng = eng;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof LynxFirmwareVersion) {
                LynxFirmwareVersion otherVersion = (LynxFirmwareVersion) other;
                return major == otherVersion.major && minor == otherVersion.minor &&
                        eng == otherVersion.eng;
            } else {
                return false;
            }
        }

        @Override
        public int compareTo(@NonNull Object another) {
            if (another instanceof LynxFirmwareVersion) {
                LynxFirmwareVersion anotherVersion = (LynxFirmwareVersion) another;
                int majorComp = Integer.compare(major, anotherVersion.major);
                if (majorComp == 0) {
                    int minorComp = Integer.compare(minor, anotherVersion.minor);
                    if (minorComp == 0) {
                        return Integer.compare(eng, anotherVersion.eng);
                    } else {
                        return minorComp;
                    }
                } else {
                    return majorComp;
                }
            }
            return 0;
        }

        @Override
        public String toString() {
            return Misc.formatInvariant("%d.%d.%d", major, minor, eng);
        }
    }

    /**
     * Retrieve and parse Lynx module firmware version.
     * @param module Lynx module
     * @return parsed firmware version
     */
    public static LynxFirmwareVersion getFirmwareVersion(LynxModule module) {
        String versionString = module.getNullableFirmwareVersionString();
        if (versionString == null) {
            return null;
        }

        String[] parts = versionString.split("[ :,]+");
        try {
            // note: for now, we ignore the hardware entry
            return new LynxFirmwareVersion(
                    Integer.parseInt(parts[3]),
                    Integer.parseInt(parts[5]),
                    Integer.parseInt(parts[7])
            );
        } catch (NumberFormatException e) {
            return null;
        }
    }

    /**
     * Exception indicating an outdated Lynx firmware version.
     */
    public static class LynxFirmwareVersionException extends RuntimeException {
        public LynxFirmwareVersionException(String detailMessage) {
            super(detailMessage);
        }
    }

    /**
     * Ensure all of the Lynx modules attached to the robot satisfy the minimum requirement.
     * @param hardwareMap hardware map containing Lynx modules
     */
    public static void ensureMinimumFirmwareVersion(HardwareMap hardwareMap) {
        Map<String, LynxFirmwareVersion> outdatedModules = new HashMap<>();
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            LynxFirmwareVersion version = getFirmwareVersion(module);
            if (version == null || version.compareTo(MIN_VERSION) < 0) {
                for (String name : hardwareMap.getNamesOf(module)) {
                    outdatedModules.put(name, version);
                }
            }
        }
        if (outdatedModules.size() > 0) {
            StringBuilder msgBuilder = new StringBuilder();
            msgBuilder.append("One or more of the attached Lynx modules has outdated firmware\n");
            msgBuilder.append(Misc.formatInvariant("Mandatory minimum firmware version: %s\n",
                    MIN_VERSION.toString()));
            for (Map.Entry<String, LynxFirmwareVersion> entry : outdatedModules.entrySet()) {
                msgBuilder.append(Misc.formatInvariant(
                        "\t%s: %s\n", entry.getKey(),
                        entry.getValue() == null ? "Unknown" : entry.getValue().toString()));
            }
            throw new LynxFirmwareVersionException(msgBuilder.toString());
        }
    }
}
