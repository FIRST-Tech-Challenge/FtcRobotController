package org.nknsd.robotics.team.programs;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.hummelvision.LilyVisionHandler;

import java.util.List;

public class VisionSensorTester extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        LilyVisionHandler lilyVisionHandler = new LilyVisionHandler();
        components.add(lilyVisionHandler);
        telemetryEnabled.add(lilyVisionHandler);
    }
}
