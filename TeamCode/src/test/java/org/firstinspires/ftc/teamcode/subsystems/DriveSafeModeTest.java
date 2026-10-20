package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Safe Mode through the real drive code: gamepad in, motor powers out.
 * The motors are mocks that remember the last power they were given, like real ones.
 */
@DisplayName("Drive in Safe Mode")
class DriveSafeModeTest {
    private static final double EPS = 1e-9;
    private final Map<String, double[]> power = new LinkedHashMap<>();
    private final BotProperties properties = new BotProperties();
    private HydraOpMode op;
    private Gamepad gamepad;
    private Imu imu;

    private DcMotorEx motor(String name) {
        double[] last = {0};
        power.put(name, last);
        DcMotorEx m = mock(DcMotorEx.class);
        doAnswer(inv -> {
            last[0] = inv.getArgument(0);
            return null;
        }).when(m).setPower(anyDouble());
        when(m.getPower()).thenAnswer(inv -> last[0]);
        return m;
    }

    @BeforeEach
    void setUp() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        for (String name : new String[] {
                properties.cfgDriveFrontLeft, properties.cfgDriveFrontRight,
                properties.cfgDriveBackLeft, properties.cfgDriveBackRight}) {
            DcMotorEx m = motor(name);
            when(hardwareMap.get(DcMotorEx.class, name)).thenReturn(m);
        }
        gamepad = mock(Gamepad.class);
        imu = mock(Imu.class);
        op = new HydraOpMode(mock(Telemetry.class), hardwareMap, gamepad, mock(Gamepad.class));
    }

    /** Run the drive loop long enough for the slew rate to reach any target. */
    private void runLoops(Drive_Manual drive) {
        for (int i = 0; i < 50; i++) {
            drive.HandleUserInput();
            drive.Process();
        }
    }

    private double maxAbsPower() {
        double max = 0;
        for (double[] p : power.values()) {
            max = Math.max(max, Math.abs(p[0]));
        }
        return max;
    }

    /** Worst case: full stick every way, boost held, heading at 45 (field-centric's 1.41). */
    private void slamEverything() {
        gamepad.left_stick_y = -1;
        gamepad.left_stick_x = 1;
        gamepad.right_stick_x = 1;
        gamepad.right_trigger = 1;
        when(imu.Ready()).thenReturn(true);
        when(imu.GetYaw()).thenReturn(45.0);
    }

    @Test
    @DisplayName("Safe Mode: no drive motor ever goes past 0.25, even at full boost")
    void safeModeCapsEveryMotor() {
        op.mSafeMode = true;
        slamEverything();
        Drive_Manual drive = new Drive_Manual(op, properties, imu, mock(Vision.class));
        for (int i = 0; i < 50; i++) {
            drive.HandleUserInput();
            drive.Process();
            assertTrue(maxAbsPower() <= BotProperties.safeModeMaxPower + EPS,
                    "loop " + i + ": " + maxAbsPower());
        }
    }

    @Test
    @DisplayName("Safe Mode: straight forward at full boost reaches exactly 0.25")
    void safeModeReachesCap() {
        op.mSafeMode = true;
        gamepad.left_stick_y = -1;
        gamepad.right_trigger = 1;
        runLoops(new Drive_Manual(op, properties, imu, mock(Vision.class)));
        for (double[] p : power.values()) {
            assertEquals(BotProperties.safeModeMaxPower, Math.abs(p[0]), EPS);
        }
    }

    @Test
    @DisplayName("full speed: the same stick goes past 0.25")
    void fullSpeedIsNotCapped() {
        op.mSafeMode = false;
        gamepad.left_stick_y = -1;
        gamepad.right_trigger = 1;
        runLoops(new Drive_Manual(op, properties, imu, mock(Vision.class)));
        assertTrue(maxAbsPower() > BotProperties.safeModeMaxPower, "max " + maxAbsPower());
    }
}
