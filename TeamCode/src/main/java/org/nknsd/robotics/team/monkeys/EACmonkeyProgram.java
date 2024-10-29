package org.nknsd.robotics.team.monkeys;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.testfiles.EACmonkey;

import java.util.List;

public class EACmonkeyProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        ExtensionHandler extensionHandler = new ExtensionHandler("motorArmExtend", true, 0.8);
        components.add(extensionHandler);
        telemetryEnabled.add(extensionHandler);

        PotentiometerHandler potentiometerHandler = new PotentiometerHandler("armPot");
        components.add(potentiometerHandler);
        telemetryEnabled.add(potentiometerHandler);

        RotationHandler rotationHandler = new RotationHandler("motorArmRotate", 0.05, 0.8, 0.02, 5, true);
        components.add(rotationHandler);
        telemetryEnabled.add(rotationHandler);

        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler("intakeServo");
        components.add(intakeSpinnerHandler);
        telemetryEnabled.add(intakeSpinnerHandler);

        EACmonkey eacMonkey = new EACmonkey();
        components.add(eacMonkey);
        telemetryEnabled.add(eacMonkey);

        eacMonkey.link(extensionHandler, intakeSpinnerHandler, rotationHandler);
        rotationHandler.link(potentiometerHandler, extensionHandler);
        extensionHandler.link(rotationHandler);
    }
}
