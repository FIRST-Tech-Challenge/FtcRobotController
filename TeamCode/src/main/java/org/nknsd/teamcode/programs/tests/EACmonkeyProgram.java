package org.nknsd.teamcode.programs.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.sensors.PotentiometerSensor;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.components.testfiles.EACmonkey;
import org.nknsd.teamcode.frameworks.NKNProgramTrue;

import java.util.List;

@TeleOp(name = "EAC Monkey", group="Tests") @Disabled
public class EACmonkeyProgram extends NKNProgramTrue {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);
        telemetryEnabled.add(extensionHandler);

        PotentiometerSensor potentiometerSensor = new PotentiometerSensor();
        components.add(potentiometerSensor);
        telemetryEnabled.add(potentiometerSensor);

        RotationHandler rotationHandler = new RotationHandler();
        components.add(rotationHandler);
        telemetryEnabled.add(rotationHandler);

        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler();
        components.add(intakeSpinnerHandler);
        telemetryEnabled.add(intakeSpinnerHandler);

        EACmonkey eacMonkey = new EACmonkey();
        components.add(eacMonkey);
        telemetryEnabled.add(eacMonkey);

        eacMonkey.link(extensionHandler, intakeSpinnerHandler, rotationHandler);
        rotationHandler.link(potentiometerSensor, extensionHandler);
        extensionHandler.link(rotationHandler);
    }
}
