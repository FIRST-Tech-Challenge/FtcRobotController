package org.nknsd.teamcode.programs.tests;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;

import java.util.List;

public class PotentiometerProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Pot
        PotentiometerSensor potentiometerSensor = new PotentiometerSensor();
        components.add(potentiometerSensor);
        telemetryEnabled.add(potentiometerSensor);
    }
}
