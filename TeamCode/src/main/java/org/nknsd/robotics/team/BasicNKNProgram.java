package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ChaosMonkey;
import org.nknsd.robotics.team.components.EventHandlerTester;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.WheelHandler;

import java.util.List;

public class BasicNKNProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        WheelHandler wheelHandler = new WheelHandler(
                "motorFL", "motorFR", "motorBL", "motorBR", new String[]{"motorFR", "motorBL", "motorFL"}
        );
        components.add(wheelHandler);

        ChaosMonkey chaosMonkey = new ChaosMonkey(wheelHandler, new String[]{});
        components.add(chaosMonkey);

        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);

        EventHandlerTester eventHandlerTester = new EventHandlerTester();
        components.add(eventHandlerTester);
        telemetryEnabled.add(eventHandlerTester);
        eventHandlerTester.link(gamePadHandler);
    }
}
