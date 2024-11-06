package org.innovators.robot.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.innovators.robot.teamcode.util.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.InOrder;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import static org.mockito.Mockito.*;

public class BasicAutonomousTest {

    private BasicAutonomous basicAutonomous;
    @Mock
    private RobotHardware mockRobotHardware;
    @Mock
    private DcMotor mockLeftDriveMotor;
    @Mock
    private DcMotor mockRightDriveMotor;
    @Mock
    private Servo mockArmWristTorqueServo;

    @BeforeEach
    @DisplayName("Test basic autonomous setup")
    public void setUp() {
        MockitoAnnotations.openMocks(this);
        basicAutonomous = spy(new BasicAutonomous());
        basicAutonomous.robot = mockRobotHardware;

        // Ensure that the robot hardware components are properly mocked
        mockRobotHardware.leftDriveMotor = mockLeftDriveMotor;
        mockRobotHardware.rightDriveMotor = mockRightDriveMotor;
        mockRobotHardware.armWristTorqueServo = mockArmWristTorqueServo;
    }

    @Test
    @DisplayName("Test runOpMode")
    public void testRunOpMode() throws InterruptedException {
        // Mock the hardware map initialization
        doNothing().when(mockRobotHardware).init(any());

        // Mock the waitForStart method to prevent it from blocking
        doNothing().when(basicAutonomous).waitForStart();

        // Run the op mode
        basicAutonomous.runOpMode();

        // Verify the sequence of operations
        InOrder inOrder = inOrder(mockRobotHardware, mockLeftDriveMotor, mockRightDriveMotor, mockArmWristTorqueServo);
        inOrder.verify(mockRobotHardware).init(any());
        inOrder.verify(mockLeftDriveMotor).setPower(Constants.DRIVE_SPEED);
        inOrder.verify(mockRightDriveMotor).setPower(Constants.DRIVE_SPEED);
        inOrder.verify(mockLeftDriveMotor).setPower(0);
        inOrder.verify(mockRightDriveMotor).setPower(0);
        inOrder.verify(mockArmWristTorqueServo).setPosition(1.0);
        inOrder.verify(mockArmWristTorqueServo).setPosition(0.0);
        inOrder.verify(mockLeftDriveMotor).setPower(-Constants.DRIVE_SPEED);
        inOrder.verify(mockRightDriveMotor).setPower(-Constants.DRIVE_SPEED);
        inOrder.verify(mockLeftDriveMotor).setPower(0);
        inOrder.verify(mockRightDriveMotor).setPower(0);
    }
}
