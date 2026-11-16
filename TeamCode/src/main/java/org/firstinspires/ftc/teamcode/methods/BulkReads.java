package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * MANUAL bulk caching: one hub command per hub per loop returns every motor encoder
 * position and velocity, instead of one command per getVelocity()/getCurrentPosition().
 *
 * Create in init() and call clear() exactly once at the START of every loop
 * (and init_loop, if it reads encoders). Without clear(), encoder values freeze.
 * The SDK resets bulk caching to OFF at the start of every OpMode, so this cannot
 * leak into an OpMode that does not use it.
 */
public final class BulkReads {
    private final List<LynxModule> hubs;

    public BulkReads(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void clear() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }
}
