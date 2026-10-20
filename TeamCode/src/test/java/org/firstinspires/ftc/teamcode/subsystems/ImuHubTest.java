package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.contains;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

@DisplayName("Heading source")
class ImuHubTest {
    private final BotProperties properties = new BotProperties();
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private HydraOpMode op;
    private IMU hubImu;

    @BeforeEach
    void setUp() {
        hardwareMap = mock(HardwareMap.class);
        telemetry = mock(Telemetry.class);
        hubImu = mock(IMU.class);
        when(hardwareMap.get(IMU.class, properties.cfgHubImu)).thenReturn(hubImu);
        op = new HydraOpMode(telemetry, hardwareMap, mock(Gamepad.class), mock(Gamepad.class));
    }

    private void hubYawReads(double degrees) {
        when(hubImu.getRobotYawPitchRollAngles())
                .thenReturn(new YawPitchRollAngles(AngleUnit.DEGREES, degrees, 0, 0, 0));
    }

    @Nested
    @DisplayName("ImuSelector")
    class Selector {

        @Test
        @DisplayName("no 'pinpoint' in the configuration: uses the Control Hub IMU")
        void noPinpointUsesHub() {
            Imu imu = ImuSelector.Pick(op, properties);
            assertInstanceOf(Imu_Hub.class, imu);
            verify(telemetry).addLine(contains("Control Hub IMU"));
        }

        @Test
        @DisplayName("'pinpoint' in the configuration: uses the Pinpoint")
        void pinpointWins() {
            GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
            when(hardwareMap.tryGet(GoBildaPinpointDriver.class, properties.cfgPinpoint))
                    .thenReturn(pinpoint);
            when(hardwareMap.get(GoBildaPinpointDriver.class, properties.cfgPinpoint))
                    .thenReturn(pinpoint);
            Imu imu = ImuSelector.Pick(op, properties);
            assertInstanceOf(Imu_Pinpoint.class, imu);
            verify(telemetry).addLine(contains("Pinpoint"));
        }
    }

    @Nested
    @DisplayName("Imu_Hub")
    class Hub {

        @Test
        @DisplayName("not ready before Init")
        void notReadyBeforeInit() {
            assertFalse(new Imu_Hub(op, properties).Ready());
        }

        @Test
        @DisplayName("Init starts the IMU once, zeroes the heading, and is ready")
        void initOnce() {
            when(hubImu.initialize(any())).thenReturn(true);
            Imu_Hub imu = new Imu_Hub(op, properties);
            assertTrue(imu.Init());
            assertTrue(imu.Init());
            assertTrue(imu.Ready());
            verify(hubImu, times(1)).initialize(any());
            verify(hubImu, times(1)).resetYaw();
        }

        @Test
        @DisplayName("IMU fails to start: INIT still finishes, heading not trusted")
        void initFailureDoesNotBlock() {
            when(hubImu.initialize(any())).thenReturn(false);
            Imu_Hub imu = new Imu_Hub(op, properties);
            assertTrue(imu.Init(), "a failed IMU must not hold up INIT");
            assertFalse(imu.Ready());
            verify(telemetry).addLine(contains("Hub IMU FAULT"));
        }

        @Test
        @DisplayName("Process reads the yaw once; GetYaw returns it")
        void readsYaw() {
            when(hubImu.initialize(any())).thenReturn(true);
            hubYawReads(37.5);
            Imu_Hub imu = new Imu_Hub(op, properties);
            imu.Init();
            imu.Process();
            assertEquals(37.5, imu.GetYaw(), 1e-9);
            imu.GetYaw();
            verify(hubImu, times(1)).getRobotYawPitchRollAngles();
        }

        @Test
        @DisplayName("not ready: Process does not touch the IMU and yaw stays 0")
        void notReadyDoesNotRead() {
            when(hubImu.initialize(any())).thenReturn(false);
            Imu_Hub imu = new Imu_Hub(op, properties);
            imu.Init();
            imu.Process();
            assertEquals(0, imu.GetYaw(), 1e-9);
            verify(hubImu, never()).getRobotYawPitchRollAngles();
        }

        @Test
        @DisplayName("ResetYaw zeroes the IMU and the stored heading")
        void resetYaw() {
            when(hubImu.initialize(any())).thenReturn(true);
            hubYawReads(90);
            Imu_Hub imu = new Imu_Hub(op, properties);
            imu.Init();
            imu.Process();
            imu.ResetYaw();
            assertEquals(0, imu.GetYaw(), 1e-9);
            verify(hubImu, times(2)).resetYaw();
        }
    }
}
