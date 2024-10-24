package org.nknsd.robotics.team.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoProgram;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.autonomous.steps.AutoStepL1;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.WheelHandler;

import java.util.List;

public class TestAuto extends NKNAutoProgram {
    private Telemetry telemetry;
    private AutoSkeleton autoSkeleton;

    @Override
    public void createSteps(List<NKNAutoStep> stepList) {
        AutoStepL1 step1 = new AutoStepL1();
        stepList.add(step1);
    }

    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Core mover
        autoSkeleton = new AutoSkeleton(0.5, 0.5);

        // Wheel Handler
        WheelHandler wheelHandler = new WheelHandler(
                "motorFL", "motorFR", "motorBL", "motorBR", new String[]{"motorFL", "motorBL"}
        );
        components.add(wheelHandler);
        telemetryEnabled.add(wheelHandler);

        // Flow Sensory Handler
        FlowSensorHandler flowSensorHandler = new FlowSensorHandler("sensor_otos", 0.590551, 3.54331, 0);
        components.add(flowSensorHandler);
        telemetryEnabled.add(flowSensorHandler);

        autoSkeleton.link(wheelHandler, flowSensorHandler);
    }
}
