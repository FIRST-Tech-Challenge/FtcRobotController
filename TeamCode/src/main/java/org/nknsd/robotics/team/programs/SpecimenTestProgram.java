package org.nknsd.robotics.team.programs;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.SpecimenClawHandler;
import org.nknsd.robotics.team.components.SpecimenExtensionHandler;
import org.nknsd.robotics.team.components.SpecimenRotationHandler;
import org.nknsd.robotics.team.components.WheelHandler;
import org.nknsd.robotics.team.components.drivers.AdvancedWheelDriver;
import org.nknsd.robotics.team.components.drivers.SpecimenDriver;

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
        IMUComponent imuComponent = new IMUComponent();
        components.add(imuComponent);

        // Driver
        AdvancedWheelDriver wheelDriver = new AdvancedWheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);

        SpecimenDriver specimenDriver = new SpecimenDriver();
        components.add(specimenDriver);
        telemetryEnabled.add(specimenDriver);



        // Link the components to each other
        wheelDriver.link(gamePadHandler, wheelHandler, imuComponent);
        specimenDriver.link(specimenExtensionHandler, specimenRotationHandler, specimenClawHandler, gamePadHandler);
    }
}
