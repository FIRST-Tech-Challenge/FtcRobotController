package org.firstinspires.ftc.teamcode.bot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.ArgumentMatchers.contains;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.PrismLedDriver;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The Bot with a pretend robot: every motor remembers the last power it was given, and a
 * motor that is "not in the robot configuration" throws, like the real hardware map does.
 */
@DisplayName("Bot")
class BotTest {
    private static final double EPS = 1e-9;

    private final BotProperties properties = new BotProperties();
    private final Map<String, double[]> power = new LinkedHashMap<>();
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private LynxModule hub;
    private IMU hubImu;

    /** Put a pretend motor in the robot configuration under this name. */
    private void addMotor(String name) {
        double[] last = {0};
        power.put(name, last);
        DcMotorEx motor = mock(DcMotorEx.class);
        doAnswer(call -> {
            last[0] = call.getArgument(0);
            return null;
        }).when(motor).setPower(anyDouble());
        when(motor.getPower()).thenAnswer(call -> last[0]);
        // doReturn, not when(...): calling get() here would throw (see setUp).
        doReturn(motor).when(hardwareMap).get(DcMotorEx.class, name);
    }

    private String[] driveMotorNames() {
        return new String[] {
                properties.cfgDriveFrontLeft, properties.cfgDriveFrontRight,
                properties.cfgDriveBackLeft, properties.cfgDriveBackRight};
    }

    private void addDrive() {
        for (String name : driveMotorNames()) {
            addMotor(name);
        }
        hubImu = mock(IMU.class);
        when(hubImu.initialize(any())).thenReturn(true);
        when(hubImu.getRobotYawPitchRollAngles())
                .thenReturn(new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
        doReturn(hubImu).when(hardwareMap).get(IMU.class, properties.cfgHubImu);
    }

    private void addLauncher() {
        addMotor(properties.cfgLaunchNectar);
    }

    /** One TeleOp loop, the way an OpMode writes it. */
    private void teleOpLoop(Bot bot) {
        bot.StartLoop();
        bot.HandleUserInput();
        bot.Process();
        bot.EndLoop();
    }

    @BeforeEach
    void setUp() {
        hardwareMap = mock(HardwareMap.class);
        // The real hardware map throws when a name is not in the robot configuration.
        when(hardwareMap.get(any(Class.class), anyString()))
                .thenThrow(new IllegalArgumentException("not in the robot configuration"));
        hub = mock(LynxModule.class);
        when(hardwareMap.getAll(LynxModule.class)).thenReturn(Collections.singletonList(hub));

        telemetry = mock(Telemetry.class);
        gamepad1 = mock(Gamepad.class);
        opMode = mock(LinearOpMode.class);
        opMode.telemetry = telemetry;
        opMode.hardwareMap = hardwareMap;
        opMode.gamepad1 = gamepad1;
        opMode.gamepad2 = mock(Gamepad.class);
    }

    @Nested
    @DisplayName("which parts are built")
    class Parts {

        @Test
        @DisplayName("normal settings: the drive and the launcher are both built")
        void everythingOn() {
            addDrive();
            addLauncher();
            new Bot(opMode, properties);
            for (String name : driveMotorNames()) {
                verify(hardwareMap).get(DcMotorEx.class, name);
            }
            verify(hardwareMap).get(DcMotorEx.class, properties.cfgLaunchNectar);
        }

        @Test
        @DisplayName("driveDisabled: works on a robot with no drive motors or heading sensor at all")
        void driveOff() {
            addLauncher();
            properties.driveDisabled = true;
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            bot.ShowStatus();
            teleOpLoop(bot);

            for (String name : driveMotorNames()) {
                verify(hardwareMap, never()).get(DcMotorEx.class, name);
            }
            verify(hardwareMap, never()).get(eq(IMU.class), anyString());
            verify(hardwareMap, never()).tryGet(eq(GoBildaPinpointDriver.class), anyString());
            verify(hardwareMap).get(DcMotorEx.class, properties.cfgLaunchNectar);
            verify(telemetry).addLine(contains("Driver control: OFF"));
        }

        @Test
        @DisplayName("launcherDisabled: works on a robot with no launcher motor at all")
        void launcherOff() {
            addDrive();
            properties.launcherDisabled = true;
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            bot.ShowStatus();
            teleOpLoop(bot);

            verify(hardwareMap, never()).get(DcMotorEx.class, properties.cfgLaunchNectar);
            verify(hardwareMap).get(DcMotorEx.class, properties.cfgDriveFrontLeft);
            verify(telemetry).addLine(contains("Launcher: OFF"));
        }

        @Test
        @DisplayName("both off (an auto): only the Safe Mode key is left, and the loop still runs")
        void bothOff() {
            properties.driveDisabled = true;
            properties.launcherDisabled = true;
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            bot.ShowStatus();
            bot.StartLoop();
            bot.Process();
            bot.EndLoop();
            verify(hardwareMap, never()).get(eq(DcMotorEx.class), anyString());
        }
    }

    @Nested
    @DisplayName("the camera")
    class Camera {

        @Test
        @DisplayName("visionDisabled: the Bot never looks for a camera, and the drive still works")
        void visionOff() {
            addDrive();
            properties.launcherDisabled = true;
            properties.visionDisabled = true;
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            bot.ShowStatus();
            teleOpLoop(bot);
            verify(hardwareMap, never()).tryGet(eq(Limelight3A.class), anyString());
            verify(hardwareMap, never()).tryGet(eq(WebcamName.class), anyString());
            verify(telemetry).addLine(contains("Vision: OFF"));
        }

        @Test
        @DisplayName("normal settings: looks for a Limelight, then a webcam, and runs without either")
        void looksForACamera() {
            addDrive();
            properties.launcherDisabled = true;
            Bot bot = new Bot(opMode, properties);
            verify(hardwareMap).tryGet(Limelight3A.class, properties.cfgLimelight);
            verify(hardwareMap).tryGet(WebcamName.class, properties.cfgWebcam);
            verify(telemetry).addLine(contains("Camera: none"));
            bot.Init();
            bot.ShowStatus();
            teleOpLoop(bot);
        }
    }

    @Nested
    @DisplayName("the indicator light")
    class Light {

        @Test
        @DisplayName("normal settings: the Bot looks for the light and follows the camera with it")
        void lightFollowsCamera() {
            properties.driveDisabled = true;
            properties.launcherDisabled = true;
            Servo light = mock(Servo.class);
            when(hardwareMap.tryGet(Servo.class, properties.cfgIndicator)).thenReturn(light);
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            bot.ShowStatus();   // waiting for START: the light already follows the camera
            // No camera in this pretend robot, so no tag: red.
            verify(light).setPosition(properties.indicatorRed);
            teleOpLoop(bot);
        }

        @Test
        @DisplayName("indicatorDisabled: the Bot never looks for the light")
        void lightOff() {
            properties.driveDisabled = true;
            properties.launcherDisabled = true;
            properties.indicatorDisabled = true;
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            teleOpLoop(bot);
            verify(hardwareMap, never()).tryGet(eq(Servo.class), anyString());
        }
    }

    @Nested
    @DisplayName("the LED strip")
    class Strip {

        @Test
        @DisplayName("normal settings: the Bot looks for the strip and runs without one")
        void looksForTheStrip() {
            properties.driveDisabled = true;
            properties.launcherDisabled = true;
            Bot bot = new Bot(opMode, properties);
            verify(hardwareMap).tryGet(PrismLedDriver.class, properties.cfgStrip);
            bot.Init();
            bot.ShowStatus();
            teleOpLoop(bot);
            verify(telemetry).addLine(contains("No light strip"));
        }

        @Test
        @DisplayName("stripDisabled: the Bot never looks for the strip")
        void stripOff() {
            properties.driveDisabled = true;
            properties.launcherDisabled = true;
            properties.stripDisabled = true;
            new Bot(opMode, properties);
            verify(hardwareMap, never()).tryGet(eq(PrismLedDriver.class), anyString());
        }
    }

    @Nested
    @DisplayName("settings")
    class Settings {

        @Test
        @DisplayName("a setting changed in the OpMode reaches the part that uses it")
        void overrideReachesThePart() {
            addLauncher();
            properties.driveDisabled = true;
            properties.launcherStartRpm = 4000;
            new Bot(opMode, properties).ShowStatus();
            verify(telemetry).addData("Launcher target RPM", "%.0f", 4000.0);
        }

        @Test
        @DisplayName("normal settings: the launcher starts at 1000 RPM")
        void normalStartSpeed() {
            addLauncher();
            properties.driveDisabled = true;
            new Bot(opMode, properties).ShowStatus();
            verify(telemetry).addData("Launcher target RPM", "%.0f", 1000.0);
        }
    }

    @Nested
    @DisplayName("Safe Mode")
    class SafeMode {

        @Test
        @DisplayName("Safe Mode switched off (today's default): the stick goes past 0.25 with no key at all")
        void switchedOffIsFullSpeed() {
            addDrive();
            properties.launcherDisabled = true;
            properties.safeModeEnabled = false;
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            gamepad1.left_stick_y = -1;
            for (int loop = 0; loop < 50; loop++) {
                teleOpLoop(bot);
            }
            assertTrue(Math.abs(power.get(properties.cfgDriveFrontLeft)[0]) > BotProperties.safeModeMaxPower);
            verify(telemetry).addLine(contains("SAFE MODE IS OFF"));
        }

        @Test
        @DisplayName("no key sensor: the stick on gamepad 1 drives, and no wheel passes 0.25")
        void driveIsLimited() {
            addDrive();
            properties.launcherDisabled = true;
            properties.safeModeEnabled = true;
            Bot bot = new Bot(opMode, properties);
            bot.Init();

            gamepad1.left_stick_y = -1;
            for (int loop = 0; loop < 50; loop++) {
                teleOpLoop(bot);
            }
            for (String name : driveMotorNames()) {
                assertEquals(BotProperties.safeModeMaxPower, Math.abs(power.get(name)[0]), EPS, name);
            }
        }

        @Test
        @DisplayName("key removed: the same stick goes past 0.25")
        void keyRemovedIsFullSpeed() {
            addDrive();
            properties.launcherDisabled = true;
            properties.safeModeEnabled = true;
            TouchSensor keySensor = mock(TouchSensor.class);
            // "Key removed" on whichever sensor is configured: see SafeModeKey.KeyIsIn.
            when(keySensor.isPressed()).thenReturn(!BotProperties.keyPressedMeansKeyIn);
            when(hardwareMap.tryGet(TouchSensor.class, BotProperties.cfgSafeModeKey)).thenReturn(keySensor);
            Bot bot = new Bot(opMode, properties);
            bot.Init();

            gamepad1.left_stick_y = -1;
            for (int loop = 0; loop < 50; loop++) {
                teleOpLoop(bot);
            }
            double frontLeft = Math.abs(power.get(properties.cfgDriveFrontLeft)[0]);
            assertTrue(frontLeft > BotProperties.safeModeMaxPower, "front left " + frontLeft);
        }

        @Test
        @DisplayName("the key is read exactly once, when the Bot is built")
        void keyReadOnce() {
            addDrive();
            addLauncher();
            properties.safeModeEnabled = true;
            TouchSensor keySensor = mock(TouchSensor.class);
            when(hardwareMap.tryGet(TouchSensor.class, BotProperties.cfgSafeModeKey)).thenReturn(keySensor);
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            for (int loop = 0; loop < 20; loop++) {
                bot.StartLoop();
                bot.ShowStatus();
                bot.EndLoop();
                teleOpLoop(bot);
            }
            verify(keySensor, times(1)).isPressed();
        }
    }

    @Nested
    @DisplayName("the loop")
    class Loop {

        @Test
        @DisplayName("no dashboard running (as here): telemetry goes to the Driver Station alone")
        void noDashboardNoWrapper() {
            assertSame(telemetry, Bot.WithDashboard(telemetry));
        }

        @Test
        @DisplayName("hubs are read in one trip per loop, and StartLoop throws the old readings away")
        void hubCaches() {
            properties.driveDisabled = true;
            properties.launcherDisabled = true;
            Bot bot = new Bot(opMode, properties);
            verify(hub).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            verify(hub, never()).clearBulkCache();
            bot.StartLoop();
            verify(hub, times(1)).clearBulkCache();
        }

        @Test
        @DisplayName("EndLoop sends the telemetry")
        void endLoopSendsTelemetry() {
            properties.driveDisabled = true;
            properties.launcherDisabled = true;
            Bot bot = new Bot(opMode, properties);
            verify(telemetry, never()).update();
            bot.EndLoop();
            verify(telemetry, times(1)).update();
        }

        @Test
        @DisplayName("an auto (Process with no HandleUserInput) leaves the launcher off")
        void autoLoopDoesNotSpinLauncher() {
            addLauncher();
            properties.driveDisabled = true;
            Bot bot = new Bot(opMode, properties);
            bot.Init();
            gamepad1.right_trigger_pressed = true;
            for (int loop = 0; loop < 5; loop++) {
                bot.StartLoop();
                bot.Process();
                bot.EndLoop();
            }
            assertEquals(0, power.get(properties.cfgLaunchNectar)[0], EPS);
        }
    }

    @Nested
    @DisplayName("Init")
    class Init {
        private GoBildaPinpointDriver pinpoint;

        /** A Pinpoint is the one part that is not ready straight away: it has to calibrate. */
        private void addDriveWithPinpoint() {
            addDrive();
            pinpoint = mock(GoBildaPinpointDriver.class);
            when(hardwareMap.tryGet(GoBildaPinpointDriver.class, properties.cfgPinpoint)).thenReturn(pinpoint);
            doReturn(pinpoint).when(hardwareMap).get(GoBildaPinpointDriver.class, properties.cfgPinpoint);
            properties.launcherDisabled = true;
        }

        @Test
        @DisplayName("every part ready the first time: Init asks once and returns")
        void readyAtOnce() {
            addDrive();
            properties.launcherDisabled = true;
            when(opMode.opModeInInit()).thenReturn(true);
            new Bot(opMode, properties).Init();
            verify(hubImu, times(1)).initialize(any());
            verify(telemetry, times(1)).update();
        }

        @Test
        @DisplayName("a part still warming up: Init keeps asking until it is ready")
        void waitsUntilReady() {
            addDriveWithPinpoint();
            when(opMode.opModeInInit()).thenReturn(true);
            when(pinpoint.getDeviceStatus()).thenReturn(
                    GoBildaPinpointDriver.DeviceStatus.CALIBRATING,
                    GoBildaPinpointDriver.DeviceStatus.CALIBRATING,
                    GoBildaPinpointDriver.DeviceStatus.READY);
            new Bot(opMode, properties).Init();
            verify(pinpoint, times(3)).getDeviceStatus();
            verify(telemetry, times(3)).update();
        }

        @Test
        @DisplayName("START or STOP pressed while a part is still warming up: Init stops waiting")
        void stopsWaitingAtStart() {
            addDriveWithPinpoint();
            when(opMode.opModeInInit()).thenReturn(true, true, false);
            when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.CALIBRATING);
            new Bot(opMode, properties).Init();
            verify(pinpoint, times(3)).getDeviceStatus();
        }

        @Test
        @DisplayName("START already pressed: every part still gets one Init")
        void initRunsAtLeastOnce() {
            addDrive();
            properties.launcherDisabled = true;
            when(opMode.opModeInInit()).thenReturn(false);
            new Bot(opMode, properties).Init();
            verify(hubImu, times(1)).initialize(any());
        }
    }
}
