package org.innovators.robot.teamcode.hardware;

import org.junit.Test;
import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotHardwareTest {

    // NOT WORKING
    @Test
    public void testHardwareInitialization() {
        RobotHardware robot = new RobotHardware();
        //HardwareMap hardwareMap = new MockHardwareMap();
        //robot.init(hardwareMap);

        assertNotNull(robot.leftDrive);
        assertNotNull(robot.rightDrive);
        assertNotNull(robot.armServo);
        assertNotNull(robot.armExtensionMotor);
        assertNotNull(robot.geckoWheelMotor);
        //assertNotNull(robot.viperSlideMotor);

        assertEquals(0.0, robot.armServo.getPosition(), 0.01);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, robot.armExtensionMotor.getMode());
    }
}
