package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Pedro's Pinpoint settings come from the same BotProperties numbers the drive uses. */
@DisplayName("PedroConstants")
class PedroConstantsTest {
    private final BotProperties properties = new BotProperties();

    @Test
    @DisplayName("the Pinpoint config names the configured Pinpoint and measures in inches")
    void pinpointConfigBasics() {
        PinpointConfig c = PedroConstants.PinpointLocalizerConfig(properties);
        assertEquals(properties.cfgPinpoint, c.name.get());
        assertEquals(DistanceUnit.INCH, c.offsetUnits.get());
        assertEquals(DistanceUnit.INCH, c.globalDistanceUnit.get());
        // The pod offsets are whatever the Pinpoint Tuner printed for this robot, so they are
        // not checked against BotProperties here. See decision 0013.
        assertTrue(Math.abs(c.xPodOffset.get()) < 12, "a pod sits within a foot of the center");
        assertTrue(Math.abs(c.yPodOffset.get()) < 12, "a pod sits within a foot of the center");
    }

    @Test
    @DisplayName("Pedro uses the pod type the tuner chose, and the drive uses the same ticks per inch")
    void podTypeAndResolutionAgree() {
        PinpointConfig c = PedroConstants.PinpointLocalizerConfig(properties);
        assertEquals(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD, c.podType.get());
        // Pedro takes ticksPerUnit over podType when both are set, so it must stay unset.
        assertFalse(c.ticksPerUnit.get().isPresent());
        // 19.894 ticks per mm, as goBILDA says for the 4-bar pod.
        assertEquals(19.894, 1 / (properties.pinpointInPerTick * 25.4), 0.001);
    }

    @Test
    @DisplayName("a 'pinpoint' in the robot configuration means the Pinpoint localizer")
    void picksPinpointWhenConfigured() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        assertFalse(PedroConstants.UsesPinpoint(hardwareMap, properties));
        when(hardwareMap.tryGet(GoBildaPinpointDriver.class, properties.cfgPinpoint))
                .thenReturn(mock(GoBildaPinpointDriver.class));
        assertTrue(PedroConstants.UsesPinpoint(hardwareMap, properties));
    }
}
