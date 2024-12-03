package org.nknsd.teamcode.programs.tests;

import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;
import org.nknsd.teamcode.components.utility.GamePadHandler;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.handlers.SpecimenClawHandler;
import org.nknsd.teamcode.components.handlers.SpecimenExtensionHandler;
import org.nknsd.teamcode.components.handlers.SpecimenRotationHandler;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.drivers.AdvancedWheelDriver;
import org.nknsd.teamcode.drivers.SpecimenDriver;

import java.util.List;

public class SpecimenTestProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Misc
        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);

        WheelHandler wheelHandler = new WheelHandler();
        components.add(wheelHandler);

        // Specimen Handlers
        SpecimenExtensionHandler specimenExtensionHandler = new SpecimenExtensionHandler();
        components.add(specimenExtensionHandler);

        SpecimenRotationHandler specimenRotationHandler = new SpecimenRotationHandler();
        components.add(specimenRotationHandler);

        SpecimenClawHandler specimenClawHandler = new SpecimenClawHandler();
        components.add(specimenClawHandler);

        // Sensors
        IMUSensor imuSensor = new IMUSensor();
        components.add(imuSensor);

        // Driver
        AdvancedWheelDriver wheelDriver = new AdvancedWheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);

        SpecimenDriver specimenDriver = new SpecimenDriver();
        components.add(specimenDriver);
        telemetryEnabled.add(specimenDriver);



        // Link the components to each other
        wheelDriver.link(gamePadHandler, wheelHandler, imuSensor);
        specimenDriver.link(specimenExtensionHandler, specimenRotationHandler, specimenClawHandler, gamePadHandler);
    }
}
