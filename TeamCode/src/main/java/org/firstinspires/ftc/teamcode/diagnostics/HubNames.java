package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Set;

/** Works out which hub (by its config name) a motor or servo controller lives on. */
final class HubNames {
    private HubNames() {}

    static String of(HardwareMap hardwareMap, HardwareDevice controller) {
        if (controller == null) return "?";
        String info = controller.getConnectionInfo();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            String hubInfo = hub.getConnectionInfo();
            if (info != null && hubInfo != null && info.startsWith(hubInfo)) {
                return firstName(hardwareMap, hub);
            }
        }
        return info == null ? "?" : info;
    }

    static String firstName(HardwareMap hardwareMap, HardwareDevice device) {
        Set<String> names = hardwareMap.getNamesOf(device);
        return names.isEmpty() ? "(unnamed)" : names.iterator().next();
    }
}
