package org.innovators.robot.teamcode.autonomous;

import static org.junit.Assert.assertEquals;

import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.junit.Test;

public class BasicAutonomousTest {

    // NOT WORKING
    @Test
    public void testAutonomousRoutine() {
        BasicAutonomous autonomousOpMode = new BasicAutonomous();
        autonomousOpMode.robot = new RobotHardware();
        autonomousOpMode.runOpMode();

        // Simulate the autonomous period
        autonomousOpMode.waitForStart();
        autonomousOpMode.loop();

        assertEquals(0.5, autonomousOpMode.robot.leftDrive.getPower(), 0.01);
        assertEquals(0.5, autonomousOpMode.robot.rightDrive.getPower(), 0.01);
    }

}
