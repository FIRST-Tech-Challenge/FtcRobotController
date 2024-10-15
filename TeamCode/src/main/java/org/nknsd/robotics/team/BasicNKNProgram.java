package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ChaosMonkey;
import org.nknsd.robotics.team.components.EventHandlerTester;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.WheelDriver;
import org.nknsd.robotics.team.components.WheelHandler;

import java.util.List;

public class BasicNKNProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        WheelHandler wheelHandler = new WheelHandler(
                "motorFL", "motorFR", "motorBL", "motorBR", new String[]{"motorFL", "motorBL"}
        );
        components.add(wheelHandler);

//        ChaosMonkey chaosMonkey = new ChaosMonkey(wheelHandler, new String[]{"MOTORBR"});
//        components.add(chaosMonkey);
//        telemetryEnabled.add(chaosMonkey);

        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);
        telemetryEnabled.add(gamePadHandler);

        WheelDriver wheelDriver = new WheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);
        wheelDriver.link(gamePadHandler, wheelHandler);
    }
}
