package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.testfiles.RotationMonkey;

import java.util.List;

public class RotatorMonkeyProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        PotentiometerHandler potentiometerHandler = new PotentiometerHandler("armPot");
        components.add(potentiometerHandler);
        telemetryEnabled.add(potentiometerHandler);

        RotationHandler rotationHandler = new RotationHandler("motorArmRotate", 0.05, 0.9);
        rotationHandler.link(potentiometerHandler);
        components.add(rotationHandler);
        telemetryEnabled.add(rotationHandler);

        RotationMonkey rotationMonkey = new RotationMonkey();
        rotationMonkey.link(rotationHandler);
        components.add(rotationMonkey);
        telemetryEnabled.add(rotationMonkey);
    }
}
