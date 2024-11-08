package org.nknsd.robotics.team.monkeys;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.testfiles.RotationMonkey;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.RotationHandler;


import java.util.List;

public class RotatorMonkeyProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Create extension handler so that the rotator can be linked to it
        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);
        telemetryEnabled.add(extensionHandler);

        // Creating components we actually want
        PotentiometerHandler potentiometerHandler = new PotentiometerHandler("armPot");
        components.add(potentiometerHandler);
        telemetryEnabled.add(potentiometerHandler);

        RotationHandler rotationHandler = new RotationHandler();
        components.add(rotationHandler);
        telemetryEnabled.add(rotationHandler);

        RotationMonkey rotatorMonkey = new RotationMonkey();
        rotatorMonkey.link(rotationHandler);
        components.add(rotatorMonkey);

        // Linking
        rotationHandler.link(potentiometerHandler, extensionHandler);
        extensionHandler.link(rotationHandler);
        telemetryEnabled.add(rotatorMonkey);
    }
}
