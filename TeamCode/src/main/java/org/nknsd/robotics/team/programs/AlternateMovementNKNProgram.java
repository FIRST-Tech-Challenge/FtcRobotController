package org.nknsd.robotics.team.programs;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.WheelHandler;
import org.nknsd.robotics.team.components.drivers.AdvancedWheelDriver;
import org.nknsd.robotics.team.components.drivers.EACDriver;

import java.util.List;

public class AlternateMovementNKNProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {
        // Misc
        GamePadHandler gamePadHandler = new GamePadHandler();
        components.add(gamePadHandler);

        WheelHandler wheelHandler = new WheelHandler(
                "motorFL", "motorFR", "motorBL", "motorBR", new String[]{"motorFL", "motorBL"}
        );
        components.add(wheelHandler);


        // Sensor
        PotentiometerHandler potentiometerHandler = new PotentiometerHandler("armPot");
        components.add(potentiometerHandler);
        //telemetryEnabled.add(potentiometerHandler);

        IMUComponent imuComponent = new IMUComponent();
        components.add(imuComponent);


        // Arm
        RotationHandler rotationHandler = new RotationHandler ("motorArmRotate", 0.05, 0.38,0.5, 0.005, 0.009,10, true);
        components.add(rotationHandler);
        //telemetryEnabled.add(rotationHandler);

        ExtensionHandler extensionHandler = new ExtensionHandler("motorArmExtend", true, 1);
        components.add(extensionHandler);
        //telemetryEnabled.add(extensionHandler);

        IntakeSpinnerHandler intakeSpinnerHandler = new IntakeSpinnerHandler("intakeServo");
        components.add(intakeSpinnerHandler);


        // Driver
        AdvancedWheelDriver wheelDriver = new AdvancedWheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);
        telemetryEnabled.add(wheelDriver);

        EACDriver eacDriver = new EACDriver();
        components.add(eacDriver);
        telemetryEnabled.add(eacDriver);


        // Link the components to each other
        wheelDriver.link(gamePadHandler, wheelHandler, imuComponent);
        rotationHandler.link(potentiometerHandler, extensionHandler);
        extensionHandler.link(rotationHandler);
        eacDriver.link(gamePadHandler, rotationHandler, extensionHandler, intakeSpinnerHandler);
    }
}
