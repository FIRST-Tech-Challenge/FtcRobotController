package org.nknsd.teamcode.programs.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;
import org.nknsd.teamcode.components.testfiles.RotationMonkey;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;


import java.util.List;

@TeleOp(name = "Rotator Monkey", group="Tests") @Disabled
public class RotatorMonkeyProgram extends NKNProgramTrue {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Create extension handler so that the rotator can be linked to it
        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);
        telemetryEnabled.add(extensionHandler);

        // Creating components we actually want
        PotentiometerSensor potentiometerSensor = new PotentiometerSensor();
        components.add(potentiometerSensor);
        telemetryEnabled.add(potentiometerSensor);

        RotationHandler rotationHandler = new RotationHandler();
        components.add(rotationHandler);
        telemetryEnabled.add(rotationHandler);

        RotationMonkey rotatorMonkey = new RotationMonkey();
        rotatorMonkey.link(rotationHandler);
        components.add(rotatorMonkey);

        // Linking
        rotationHandler.link(potentiometerSensor, extensionHandler);
        extensionHandler.link(rotationHandler);
        telemetryEnabled.add(rotatorMonkey);
    }
}
