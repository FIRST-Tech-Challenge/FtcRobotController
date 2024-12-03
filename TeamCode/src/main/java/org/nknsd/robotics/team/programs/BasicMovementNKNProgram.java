package org.nknsd.robotics.team.programs;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.drivers.AdvancedWheelDriver;
import org.nknsd.robotics.team.components.WheelHandler;
import org.nknsd.robotics.team.components.drivers.EACDriver;
import org.nknsd.robotics.team.components.drivers.WheelDriver;
import org.nknsd.robotics.team.controlSchemes.reals.CollyWheelController;
import org.nknsd.robotics.team.controlSchemes.reals.KarstenEACController;

import java.util.List;

public class BasicMovementNKNProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Misc
        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);
        //telemetryEnabled.add(gamePadHandler);

        WheelHandler wheelHandler = new WheelHandler();
        components.add(wheelHandler);


        // Sensor
        PotentiometerHandler potentiometerHandler = new PotentiometerHandler();
        components.add(potentiometerHandler);
        //telemetryEnabled.add(potentiometerHandler);

        IMUComponent imuComponent = new IMUComponent();
        components.add(imuComponent);


        // Arm
        RotationHandler rotationHandler = new RotationHandler();
        components.add(rotationHandler);
        //telemetryEnabled.add(rotationHandler);

        ExtensionHandler extensionHandler = new ExtensionHandler();
        components.add(extensionHandler);
        //telemetryEnabled.add(extensionHandler);

        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler();
        components.add(intakeSpinnerHandler);


        // Driver
        WheelDriver wheelDriver = new WheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);

        EACDriver eacDriver = new EACDriver();
        components.add(eacDriver);
        telemetryEnabled.add(eacDriver);


        // Controllers
        CollyWheelController wheelController = new CollyWheelController();
        wheelController.link(gamePadHandler);

        KarstenEACController eacController = new KarstenEACController();
        eacController.link(gamePadHandler);
        eacController.linkExtensionHandler(extensionHandler);


        // Link the components to each other
        wheelDriver.link(gamePadHandler, wheelHandler, wheelController);
        rotationHandler.link(potentiometerHandler, extensionHandler);
        extensionHandler.link(rotationHandler);
        eacDriver.link(gamePadHandler, rotationHandler, extensionHandler, intakeSpinnerHandler, eacController);
    }
}
