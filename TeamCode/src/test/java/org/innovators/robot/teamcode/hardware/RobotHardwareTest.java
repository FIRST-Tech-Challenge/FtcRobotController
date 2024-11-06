package org.innovators.robot.teamcode.hardware;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.junit.jupiter.api.DisplayName;

public class RobotHardwareTest {
    private RobotHardware mockRobot;
    private HardwareMap mockHardwareMap;

    @BeforeEach
    @DisplayName("Test robot hardware setup")
    public void setUp() {
        // Initialize the RobotHardware instance
        mockRobot = new RobotHardware();

        // Mock the HardwareMap and its components
        mockHardwareMap = mock(HardwareMap.class);
        DcMotor mockLeftDriveMotor = mock(DcMotor.class);
        DcMotor mockRightDriveMotor = mock(DcMotor.class);
        DcMotor mockArmExtensionMotor = mock(DcMotor.class);
        Servo mockArmWristTorqueServo = mock(Servo.class);
        Servo mockGeckoWheelSpeedServo = mock(Servo.class);

        // Define behavior for the mock objects
        when(mockHardwareMap.get(DcMotor.class, "left_drive_motor")).thenReturn(mockLeftDriveMotor);
        when(mockHardwareMap.get(DcMotor.class, "right_drive_motor")).thenReturn(mockRightDriveMotor);
        when(mockHardwareMap.get(DcMotor.class, "arm_extension_motor")).thenReturn(mockArmExtensionMotor);
        when(mockHardwareMap.get(Servo.class, "arm_wrist_torque_servo")).thenReturn(mockArmWristTorqueServo);
        when(mockHardwareMap.get(Servo.class, "gecko_wheel_speed_servo")).thenReturn(mockGeckoWheelSpeedServo);
    }

    @Test
    @DisplayName("Test hardware initialization")
    public void testMotorInitialization() {
        mockRobot.init(mockHardwareMap);

        assertNotNull(mockRobot.leftDriveMotor, "Left drive motor should be initialized");
        assertNotNull(mockRobot.rightDriveMotor, "Right drive motor should be initialized");
    }
}
