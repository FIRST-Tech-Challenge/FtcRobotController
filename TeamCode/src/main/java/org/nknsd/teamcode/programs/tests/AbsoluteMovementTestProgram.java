package org.nknsd.teamcode.programs.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.drivers.AdvancedWheelDriver;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;

import java.util.List;

@TeleOp(name = "Advanced Movement Test", group="Tests")
public class AbsoluteMovementTestProgram extends NKNProgramTrue {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Gamepad Handler
        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);
        //telemetryEnabled.add(gamePadHandler);

        // Wheel Handler
        WheelHandler wheelHandler = new WheelHandler();
        components.add(wheelHandler);
        //telemetryEnabled.add(wheelHandler);

        // Flow Sensory Handler
        FlowSensor flowSensor = new FlowSensor();
        components.add(flowSensor);
        //telemetryEnabled.add(flowSensorHandler);

        // IMU Handler
        IMUSensor imuSensor = new IMUSensor();
        components.add(imuSensor);
        //telemetryEnabled.add(imuComponent);

        // Wheel Driver
        AdvancedWheelDriver wheelDriver = new AdvancedWheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        //telemetryEnabled.add(wheelDriver);
        wheelDriver.link(gamePadHandler, wheelHandler, imuSensor);
    }
}
