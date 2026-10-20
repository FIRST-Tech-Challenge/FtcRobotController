package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/**
 * Pedro's drivetrain with the team's Safe Mode limit. Uses Pedro's real Mecanum math with
 * pretend motors that remember the last power they were given.
 */
@DisplayName("SafeModeMecanum")
class SafeModeMecanumTest {
    private final Map<String, DcMotorEx> motors = new HashMap<>();
    private final Map<String, Double> lastPower = new HashMap<>();
    private final BotProperties properties = new BotProperties();
    private HardwareMap hardwareMap;

    @BeforeEach
    void setUp() {
        hardwareMap = mock(HardwareMap.class);
        String[] names = {
                properties.cfgDriveFrontLeft, properties.cfgDriveFrontRight,
                properties.cfgDriveBackLeft, properties.cfgDriveBackRight};
        for (String name : names) {
            DcMotorEx motor = mock(DcMotorEx.class);
            lastPower.put(name, 0.0);
            doAnswer(call -> {
                lastPower.put(name, call.getArgument(0));
                return null;
            }).when(motor).setPower(anyDouble());
            when(hardwareMap.get(DcMotorEx.class, name)).thenReturn(motor);
            motors.put(name, motor);
        }
    }

    private double biggestPower() {
        double biggest = 0;
        for (double power : lastPower.values()) {
            biggest = Math.max(biggest, Math.abs(power));
        }
        return biggest;
    }

    private SafeModeMecanum drivetrain(boolean safeMode) {
        return new SafeModeMecanum(hardwareMap, PedroConstants.DriveConfig(properties), safeMode);
    }

    @ParameterizedTest(name = "forward {0}, strafe {1}, turn {2}")
    @CsvSource({
            "1, 0, 0",
            "0, 1, 0",
            "0, 0, 1",
            "1, 1, 1",
            "-1, -1, -1",
            "5, -3, 2",
            "0.1, 0, 0"})
    @DisplayName("Safe Mode: no wheel ever goes past 0.25")
    void safeModeCaps(double forward, double strafe, double turn) {
        drivetrain(true).drive(new DrivePowers(forward, strafe, turn), false);
        assertTrue(biggestPower() <= BotProperties.safeModeMaxPower + 1e-9,
                "biggest wheel power was " + biggestPower());
    }

    @Test
    @DisplayName("Safe Mode: full forward runs every wheel at exactly 0.25")
    void safeModeFullForward() {
        drivetrain(true).drive(new DrivePowers(1, 0, 0), false);
        for (Map.Entry<String, Double> wheel : lastPower.entrySet()) {
            assertEquals(BotProperties.safeModeMaxPower, Math.abs(wheel.getValue()), 1e-9, wheel.getKey());
        }
    }

    @Test
    @DisplayName("Safe Mode: the robot still moves the same direction, just slower")
    void safeModeKeepsDirection() {
        DrivePowers diagonal = new DrivePowers(1, 0.5, 0.2);
        drivetrain(false).drive(diagonal, false);
        Map<String, Double> fullSpeed = new HashMap<>(lastPower);

        drivetrain(true).drive(diagonal, false);
        double ratio = lastPower.get(properties.cfgDriveFrontLeft) / fullSpeed.get(properties.cfgDriveFrontLeft);
        for (String name : lastPower.keySet()) {
            assertEquals(fullSpeed.get(name) * ratio, lastPower.get(name), 1e-9, name);
        }
    }

    @Test
    @DisplayName("full speed: Pedro's powers pass through unchanged")
    void fullSpeedUnchanged() {
        drivetrain(false).drive(new DrivePowers(1, 0, 0), false);
        assertEquals(1.0, biggestPower(), 1e-9);
    }

    @Test
    @DisplayName("every drive motor runs without its encoder (two of them carry the odometry pods)")
    void runWithoutEncoder() {
        drivetrain(true);
        for (DcMotorEx motor : motors.values()) {
            verify(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
