package org.innovators.robot.teamcode;

import org.innovators.robot.teamcode.autonomous.BasicAutonomousTest;
import org.innovators.robot.teamcode.hardware.RobotHardwareTest;
import org.junit.platform.suite.api.SelectClasses;
import org.junit.platform.suite.api.Suite;

@Suite
@SelectClasses({
    BasicAutonomousTest.class,
    RobotHardwareTest.class
})
public class TestRunner {
}
