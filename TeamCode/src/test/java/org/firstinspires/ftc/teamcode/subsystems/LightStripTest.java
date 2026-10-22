package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.AdditionalMatchers.aryEq;
import static org.mockito.ArgumentMatchers.contains;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drivers.PrismLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** The strip with a pretend Prism driver: we check exactly which bytes it is sent. */
@DisplayName("LightStrip (Prism driver)")
class LightStripTest {
    private final BotProperties properties = new BotProperties();
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private PrismLedDriver prism;
    private HydraOpMode op;

    @BeforeEach
    void setUp() {
        hardwareMap = mock(HardwareMap.class);
        telemetry = mock(Telemetry.class);
        gamepad = mock(Gamepad.class);
        prism = mock(PrismLedDriver.class);
        when(prism.ReadStatus()).thenReturn(new byte[] {12, 60, 0, 0});
        op = new HydraOpMode(telemetry, hardwareMap, gamepad, mock(Gamepad.class));
    }

    private LightStrip stripConfigured() {
        when(hardwareMap.tryGet(PrismLedDriver.class, properties.cfgStrip)).thenReturn(prism);
        return new LightStrip(op, properties, gamepad);
    }

    @Test
    @DisplayName("Init reads the device id and status once, and shows them")
    void initOnce() {
        LightStrip strip = stripConfigured();
        strip.Init();
        strip.Init();
        verify(prism, times(1)).ReadDeviceId();
        verify(prism, times(1)).ReadStatus();
        verify(telemetry, times(2)).addLine(contains("D-pad: up red"));
    }

    @Test
    @DisplayName("solid red: the whole strip, layer 0, at the set brightness")
    void solidRed() {
        properties.stripLedCount = 12;
        properties.stripBrightness = 50;
        LightStrip strip = stripConfigured();
        strip.Solid(255, 0, 0);
        verify(prism).SolidColor(0, 0, 12, 50, 255, 0, 0);
    }

    @Test
    @DisplayName("off: layer 0 gets animation 0")
    void off() {
        LightStrip strip = stripConfigured();
        strip.Off();
        verify(prism).ClearLayer(0);
    }

    @Test
    @DisplayName("a D-pad press sends one color, however long it is held")
    void dpadSendsOnce() {
        LightStrip strip = stripConfigured();
        gamepad.dpad_right = true;
        for (int loop = 0; loop < 30; loop++) {
            strip.HandleUserInput();
        }
        verify(prism, times(1)).SolidColor(0, 0, properties.stripLedCount, properties.stripBrightness, 0, 255, 0);
    }

    @Test
    @DisplayName("a 4-byte value goes lowest byte first")
    void bytes32() {
        assertArrayEquals(new byte[] {(byte) 0xD0, 0x07, 0, 0}, LightStrip.Bytes32(2000));
        assertArrayEquals(new byte[] {1, 0, 0, 0}, LightStrip.Bytes32(1));
    }

    @Test
    @DisplayName("no strip in the robot configuration: nothing is sent, nothing crashes, INIT says so")
    void noStrip() {
        LightStrip strip = new LightStrip(op, properties, gamepad);
        strip.Init();
        strip.Solid(255, 255, 255);
        strip.Process();
        verify(prism, never()).SolidColor(anyInt(), anyInt(), anyInt(), anyInt(), anyInt(), anyInt(), anyInt());
        verify(prism, never()).WriteLayer(anyInt(), anyInt(), any(byte[].class));
        verify(telemetry).addLine(contains("No light strip"));
    }
}
