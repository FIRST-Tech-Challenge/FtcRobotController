package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Function;
import java.util.function.Supplier;
import java.util.Arrays;

/** Name-only resolution; the inventory supplier is diagnostic data, never a device selector. */
public final class NamedHardware {
    private NamedHardware() { }
    public static <T> T require(Function<String,T> namedLookup, Supplier<String> diagnostics, String... supportedNames) {
        for (String name : supportedNames) {
            T device=namedLookup.apply(name);
            if (device != null) return device;
        }
        throw new IllegalArgumentException("Missing named hardware " + Arrays.toString(supportedNames) + ". Configured devices: " + diagnostics.get());
    }

    /** Motor lookup by name only; on failure lists every configured motor (quoted, with hub and port). */
    public static DcMotorEx requireMotor(HardwareMap hwMap, String... supportedNames) {
        return require(name -> hwMap.tryGet(DcMotorEx.class, name), () -> motorInventory(hwMap), supportedNames);
    }

    /** Quoted names expose stray spaces or capitals that the Driver Hub config screen hides. */
    static String motorInventory(HardwareMap hwMap) {
        StringBuilder configured = new StringBuilder("motors: ");
        for (DcMotorEx motor : hwMap.getAll(DcMotorEx.class)) {
            for (String name : hwMap.getNamesOf(motor)) configured.append('"').append(name).append("\" ");
            configured.append('[').append(motor.getController().getConnectionInfo())
                    .append(", port ").append(motor.getPortNumber()).append("]; ");
        }
        return configured.toString();
    }
}
