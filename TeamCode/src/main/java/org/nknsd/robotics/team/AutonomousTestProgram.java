package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.WheelDriver;
import org.nknsd.robotics.team.components.WheelHandler;

import java.util.List;

public class AutonomousTestProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        WheelHandler wheelHandler = new WheelHandler(
                "motorFL", "motorFR", "motorBL", "motorBR", new String[]{"motorFL", "motorBL"}
        );
        components.add(wheelHandler);
        telemetryEnabled.add(wheelHandler);

        FlowSensorHandler flowSensorHandler = new FlowSensorHandler("sensor_otos", 0, 0, 0);
        components.add(flowSensorHandler);
        telemetryEnabled.add(flowSensorHandler);

        AutoSkeleton autoSkeleton = new AutoSkeleton(0.5, 0.1);
        autoSkeleton.link(wheelHandler, flowSensorHandler);


    }
}
