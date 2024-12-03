package org.nknsd.teamcode.programs.tests;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.drivers.WheelDriver;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.controlSchemes.reals.CollyWheelController;

import java.util.List;

public class FlowSensorTestProgram extends NKNProgram {
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
        telemetryEnabled.add(flowSensor);

        // IMU Handler
        IMUSensor imuSensor = new IMUSensor();
        components.add(imuSensor);
        telemetryEnabled.add(imuSensor);

        // Wheel Driver
        WheelDriver wheelDriver = new WheelDriver(0, 1, 10, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);

        CollyWheelController wheelController = new CollyWheelController();
        wheelController.link(gamePadHandler);

        wheelDriver.link(gamePadHandler, wheelHandler, wheelController);
    }
}
