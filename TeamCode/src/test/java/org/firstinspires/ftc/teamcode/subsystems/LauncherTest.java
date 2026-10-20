package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * The launcher with a pretend motor and gamepad. The motor remembers the last power it
 * was given, and reports whatever speed the test sets.
 */
@DisplayName("Launcher")
class LauncherTest {
    private static final double EPS = 1e-9;
    private final BotProperties properties = new BotProperties();
    private final double UPDATE_SECONDS = properties.launcherUpdateMs / 1000;

    private DcMotorEx motor;
    private Gamepad gamepad;
    private HydraOpMode op;
    private Launcher launcher;
    private final double[] lastPower = {0};
    private final double[] ticksPerSecond = {0};

    @BeforeEach
    void setUp() {
        motor = mock(DcMotorEx.class);
        doAnswer(inv -> {
            lastPower[0] = inv.getArgument(0);
            return null;
        }).when(motor).setPower(anyDouble());
        when(motor.getVelocity()).thenAnswer(inv -> ticksPerSecond[0]);

        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(DcMotorEx.class, properties.cfgLaunchNectar)).thenReturn(motor);
        gamepad = mock(Gamepad.class);
        op = new HydraOpMode(mock(Telemetry.class), hardwareMap, gamepad, gamepad);
        launcher = new Launcher(op, properties, gamepad);
    }

    /** Press a bumper and hold it long enough for the debouncer, then let go. */
    private void tapBumper(boolean right) {
        for (int loop = 0; loop < properties.debounce + 2; loop++) {
            gamepad.right_bumper = right;
            gamepad.left_bumper = !right;
            launcher.HandleUserInput();
        }
        gamepad.right_bumper = false;
        gamepad.left_bumper = false;
        launcher.HandleUserInput();
    }

    /** Pretend the wheel is spinning at this RPM. */
    private void wheelSpinsAt(double rpm) {
        ticksPerSecond[0] = rpm / 60 * properties.launcherTicksPerRev;
    }

    @Test
    @DisplayName("motor is set up like Decode: our code picks power, wheel coasts when off")
    void motorSetUp() {
        verify(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Test
    @DisplayName("target starts at 1000 RPM")
    void startsAt1000() {
        assertEquals(1000, launcher.GetTargetRpm(), EPS);
    }

    @Test
    @DisplayName("an OpMode's own start speed is used, and the normal one is left alone")
    void startSpeedOverride() {
        BotProperties fasterStart = new BotProperties();
        fasterStart.launcherStartRpm = 4000;
        Launcher fastLauncher = new Launcher(op, fasterStart, gamepad);
        assertEquals(4000, fastLauncher.GetTargetRpm(), EPS);
        assertEquals(1000, new BotProperties().launcherStartRpm, EPS);
    }

    @Test
    @DisplayName("right bumper adds 50 RPM, left bumper takes away 50, once per press")
    void bumpersStep50() {
        tapBumper(true);
        assertEquals(1050, launcher.GetTargetRpm(), EPS);
        tapBumper(true);
        assertEquals(1100, launcher.GetTargetRpm(), EPS);
        tapBumper(false);
        assertEquals(1050, launcher.GetTargetRpm(), EPS);
    }

    @Test
    @DisplayName("target never goes below 0 or above the maximum")
    void targetLimits() {
        launcher.ChangeTargetRpm(-5000);
        assertEquals(0, launcher.GetTargetRpm(), EPS);
        launcher.ChangeTargetRpm(properties.launcherMaxRpm + 1000);
        assertEquals(properties.launcherMaxRpm, launcher.GetTargetRpm(), EPS);
    }

    @Test
    @DisplayName("no trigger: no power, even below the target")
    void noTriggerNoPower() {
        gamepad.right_trigger_pressed = false;
        wheelSpinsAt(0);
        launcher.HandleUserInput();
        launcher.Update(UPDATE_SECONDS);
        assertEquals(0, lastPower[0], EPS);
    }

    @Test
    @DisplayName("trigger held: power comes from the speed control")
    void triggerGivesPower() {
        gamepad.right_trigger_pressed = true;
        wheelSpinsAt(900);
        launcher.HandleUserInput();
        launcher.Update(UPDATE_SECONDS);
        double expected = properties.launcherKF * 1000 + properties.launcherKP * 100
                + properties.launcherKI * 100 * UPDATE_SECONDS;
        assertEquals(Math.min(expected, 1), lastPower[0], EPS);
    }

    @Test
    @DisplayName("letting go of the trigger turns the power off")
    void releaseTrigger() {
        gamepad.right_trigger_pressed = true;
        launcher.HandleUserInput();
        launcher.Update(UPDATE_SECONDS);
        assertTrue(lastPower[0] > 0);

        gamepad.right_trigger_pressed = false;
        launcher.HandleUserInput();
        launcher.Update(UPDATE_SECONDS);
        assertEquals(0, lastPower[0], EPS);
    }

    @Test
    @DisplayName("RPM: 28 ticks per second on a 28-tick motor is 60 RPM")
    void rpmMath() {
        ticksPerSecond[0] = properties.launcherTicksPerRev;
        launcher.Update(UPDATE_SECONDS);
        assertEquals(60, launcher.GetMeasuredRpm(), EPS);
    }

    @Test
    @DisplayName("at speed within 50 RPM of the target, not outside it")
    void atSpeed() {
        wheelSpinsAt(1040);
        launcher.Update(UPDATE_SECONDS);
        assertTrue(launcher.AtSpeed());
        wheelSpinsAt(900);
        launcher.Update(UPDATE_SECONDS);
        assertFalse(launcher.AtSpeed());
    }
}
