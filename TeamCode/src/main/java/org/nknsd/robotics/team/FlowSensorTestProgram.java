package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.drivers.WheelDriver;
import org.nknsd.robotics.team.components.WheelHandler;

import java.util.List;

public class FlowSensorTestProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Gamepad Handler
        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);
        //telemetryEnabled.add(gamePadHandler);

        // Wheel Handler
        WheelHandler wheelHandler = new WheelHandler(
                "motorFL", "motorFR", "motorBL", "motorBR", new String[]{"motorFL", "motorBL"}
        );
        components.add(wheelHandler);
        //telemetryEnabled.add(wheelHandler);

        // Flow Sensory Handler
        FlowSensorHandler flowSensorHandler = new FlowSensorHandler("sensor_otos", 0.590551, 3.54331, 0);
        components.add(flowSensorHandler);
        telemetryEnabled.add(flowSensorHandler);

        // IMU Handler
        IMUComponent imuComponent = new IMUComponent();
        components.add(imuComponent);
        telemetryEnabled.add(imuComponent);

        // Wheel Driver
        WheelDriver wheelDriver = new WheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);
        wheelDriver.link(gamePadHandler, wheelHandler);
    }
}
