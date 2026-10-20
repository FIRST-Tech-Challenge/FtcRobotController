package org.firstinspires.ftc.teamcode.subsystems;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.contains;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.atLeast;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** The light with a pretend servo and a pretend camera. */
@DisplayName("Indicator light")
class IndicatorTest {
    private final BotProperties properties = new BotProperties();
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private HydraOpMode op;
    private Servo light;
    private Vision vision;

    @BeforeEach
    void setUp() {
        hardwareMap = mock(HardwareMap.class);
        telemetry = mock(Telemetry.class);
        light = mock(Servo.class);
        vision = mock(Vision.class);
        op = new HydraOpMode(telemetry, hardwareMap, mock(Gamepad.class), mock(Gamepad.class));
    }

    private Indicator lightConfigured() {
        when(hardwareMap.tryGet(Servo.class, properties.cfgIndicator)).thenReturn(light);
        return new Indicator(op, properties, vision);
    }

    @Test
    @DisplayName("three or more tags in view: solid green")
    void greenForThree() {
        Indicator indicator = lightConfigured();
        when(vision.SeesTag()).thenReturn(true);
        when(vision.CountTags()).thenReturn(3);
        for (int loop = 0; loop < 20; loop++) {
            indicator.Process();
        }
        verify(light).setPosition(properties.indicatorGreen);
        verify(light, never()).setPosition(properties.indicatorBlue);
        verify(light, times(1)).setPosition(properties.indicatorOff);   // only the "off" from being built
    }

    @Test
    @DisplayName("two tags in view: solid blue")
    void blueForTwo() {
        Indicator indicator = lightConfigured();
        when(vision.SeesTag()).thenReturn(true);
        when(vision.CountTags()).thenReturn(2);
        for (int loop = 0; loop < 20; loop++) {
            indicator.Process();
        }
        verify(light).setPosition(properties.indicatorBlue);
        verify(light, never()).setPosition(properties.indicatorGreen);
        verify(light, times(1)).setPosition(properties.indicatorOff);
    }

    @Test
    @DisplayName("exactly one tag: blinks, blue and off taking turns")
    void blinksForOne() {
        properties.indicatorBlinkMs = 0;   // every loop is a new half of the blink
        Indicator indicator = lightConfigured();
        when(vision.SeesTag()).thenReturn(true);
        when(vision.CountTags()).thenReturn(1);
        for (int loop = 0; loop < 6; loop++) {
            indicator.Process();
        }
        verify(light, atLeast(3)).setPosition(properties.indicatorBlue);
        verify(light, atLeast(3)).setPosition(properties.indicatorOff);
        verify(light, never()).setPosition(properties.indicatorRed);
        verify(light, never()).setPosition(properties.indicatorGreen);
    }

    @Test
    @DisplayName("one tag, blink still on its first half: blue, not off")
    void blinkStartsBlue() {
        Indicator indicator = lightConfigured();
        when(vision.SeesTag()).thenReturn(true);
        when(vision.CountTags()).thenReturn(1);
        indicator.Process();
        verify(light).setPosition(properties.indicatorBlue);
    }

    @Test
    @DisplayName("no tag: red")
    void redWhenNot() {
        Indicator indicator = lightConfigured();
        when(vision.SeesTag()).thenReturn(false);
        indicator.Process();
        verify(light).setPosition(properties.indicatorRed);
    }

    @Test
    @DisplayName("off when built, before anything is seen")
    void offAtFirst() {
        lightConfigured();
        verify(light).setPosition(properties.indicatorOff);
    }

    @Test
    @DisplayName("the same color is sent once, not fifty times a second")
    void sendsOnlyChanges() {
        Indicator indicator = lightConfigured();
        when(vision.SeesTag()).thenReturn(true);
        when(vision.CountTags()).thenReturn(3);
        for (int loop = 0; loop < 50; loop++) {
            indicator.Process();
        }
        verify(light, times(1)).setPosition(properties.indicatorGreen);
    }

    @Test
    @DisplayName("no light in the robot configuration: nothing crashes, and INIT says so")
    void noLightConfigured() {
        Indicator indicator = new Indicator(op, properties, vision);
        indicator.Init();
        when(vision.SeesTag()).thenReturn(true);
        indicator.Process();
        verify(light, never()).setPosition(anyDouble());
        verify(telemetry).addLine(contains("No indicator light"));
    }
}
