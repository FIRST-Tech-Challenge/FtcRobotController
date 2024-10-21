package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ArmRotator;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.testfiles.ExtensionMonkey;
import org.nknsd.robotics.team.components.testfiles.RotatorMonkey;

import java.util.List;

public class ExtensionMonkeyProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        ExtensionHandler extensionHandler = new ExtensionHandler("motorArmExtend", true, 0.8);
        components.add(extensionHandler);
        telemetryEnabled.add(extensionHandler);

        ExtensionMonkey extensionMonkey = new ExtensionMonkey();
        extensionMonkey.link(extensionHandler);
        components.add(extensionMonkey);
        telemetryEnabled.add(extensionMonkey);
    }
}
