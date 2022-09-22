package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.HashMap;
import java.util.Map;

/**
 * Parsed representation of a Lynx module firmware version.
 */
public final class LynxFirmwareVersion implements Comparable<LynxFirmwareVersion> {

    /**
     * Ensure all of the Lynx modules attached to the robot satisfy the minimum requirement.
     * @param hardwareMap hardware map containing Lynx modules
     */
    public static void throwIfAnyModulesBelowVersion(HardwareMap hardwareMap, LynxFirmwareVersion minVersion) {
        Map<String, LynxFirmwareVersion> outdatedModules = new HashMap<>();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            LynxFirmwareVersion version = LynxFirmwareVersion.parse(module.getNullableFirmwareVersionString());
            if (version == null || version.compareTo(minVersion) < 0) {
                for (String name : hardwareMap.getNamesOf(module)) {
                    outdatedModules.put(name, version);
                }
            }
        }

        if (outdatedModules.size() > 0) {
            StringBuilder msgBuilder = new StringBuilder();
            msgBuilder.append("One or more of the attached Lynx modules has outdated firmware\n");
            msgBuilder.append(Misc.formatInvariant("Mandatory minimum firmware version for Road Runner: %s\n",
                    minVersion.toString()));

            for (Map.Entry<String, LynxFirmwareVersion> entry : outdatedModules.entrySet()) {
                msgBuilder.append(Misc.formatInvariant(
                        "\t%s: %s\n", entry.getKey(),
                        entry.getValue() == null ? "Unknown" : entry.getValue().toString()));
            }

            throw new RuntimeException(msgBuilder.toString());
        }
    }

    /**
     * Parses Lynx module firmware version string into structured version object. Returns null for
     * null argument or upon error.
     */
    public static LynxFirmwareVersion parse(String s) {
        if (s == null) {
            return null;
        }

        String[] parts = s.split("[ :,]+");
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

    public final int major, minor, eng;

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
    public int compareTo(LynxFirmwareVersion other) {
        int majorComp = Integer.compare(major, other.major);
        if (majorComp == 0) {
            int minorComp = Integer.compare(minor, other.minor);
            if (minorComp == 0) {
                return Integer.compare(eng, other.eng);
            } else {
                return minorComp;
            }
        } else {
            return majorComp;
        }
    }

    @Override
    public String toString() {
        return Misc.formatInvariant("%d.%d.%d", major, minor, eng);
    }
}
