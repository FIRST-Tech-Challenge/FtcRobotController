package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.mockito.ArgumentMatchers.contains;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Which camera gets picked. The webcam case builds a VisionPortal, which needs Android's
 * camera stack, so it is checked on the robot, not here.
 */
@DisplayName("VisionSelector")
class VisionSelectorTest {
    private final BotProperties properties = new BotProperties();
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private HydraOpMode op;

    @BeforeEach
    void setUp() {
        hardwareMap = mock(HardwareMap.class);
        telemetry = mock(Telemetry.class);
        op = new HydraOpMode(telemetry, hardwareMap, mock(Gamepad.class), mock(Gamepad.class));
    }

    @Test
    @DisplayName("a Limelight in the robot configuration wins")
    void limelightWins() {
        Limelight3A limelight = mock(Limelight3A.class);
        when(hardwareMap.tryGet(Limelight3A.class, properties.cfgLimelight)).thenReturn(limelight);
        when(hardwareMap.get(Limelight3A.class, properties.cfgLimelight)).thenReturn(limelight);
        Vision vision = VisionSelector.Pick(op, properties);
        assertInstanceOf(Vision_Limelight.class, vision);
        verify(telemetry).addLine(contains("Limelight"));
    }

    @Test
    @DisplayName("no camera at all: a Vision that never sees a tag, and a note on the Driver Station")
    void noCamera() {
        Vision vision = VisionSelector.Pick(op, properties);
        assertInstanceOf(Vision_None.class, vision);
        assertFalse(vision.SeesTag());
        vision.Init();
        verify(telemetry).addLine(contains("No camera"));
    }
}
