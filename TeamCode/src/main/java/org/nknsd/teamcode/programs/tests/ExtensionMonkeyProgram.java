package org.nknsd.teamcode.programs.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.components.testfiles.ExtensionMonkey;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;

import java.util.List;

@TeleOp(name = "Extension Monkey", group="Tests") @Disabled
public class ExtensionMonkeyProgram extends NKNProgramTrue {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Create pot and arm rotator so that the extensionHandler can be linked to them
        PotentiometerSensor potentiometerSensor = new PotentiometerSensor();
        components.add(potentiometerSensor);

        RotationHandler rotationHandler = new RotationHandler();
        components.add(rotationHandler);

        // Actual components we actually use
        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);
        telemetryEnabled.add(extensionHandler);

        rotationHandler.link(potentiometerSensor, extensionHandler);
        extensionHandler.link(rotationHandler);

        ExtensionMonkey extensionMonkey = new ExtensionMonkey();
        extensionMonkey.link(extensionHandler);
        components.add(extensionMonkey);
        telemetryEnabled.add(extensionMonkey);
    }
}
