package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

/**
 * Picks where the heading comes from, based on what is in the robot configuration.
 *
 * <p>A Pinpoint named "pinpoint" wins: it is more accurate and will also give position
 * later. Without one, use the IMU inside the Control Hub, named "imu". Every Control Hub
 * configuration has "imu" by default. See docs/decisions/0005.
 */
public final class ImuSelector {
    private ImuSelector() {
    }

    public static Imu Pick(HydraOpMode op, BotProperties properties) {
        if (op.mHardwareMap.tryGet(GoBildaPinpointDriver.class, properties.cfgPinpoint) != null) {
            op.mTelemetry.addLine("Heading from: Pinpoint");
            return new Imu_Pinpoint(op, properties);
        }
        op.mTelemetry.addLine("Heading from: Control Hub IMU (no '" + properties.cfgPinpoint
                + "' in the robot configuration)");
        return new Imu_Hub(op, properties);
    }
}
