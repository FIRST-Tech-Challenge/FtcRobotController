package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.GamePadHandler;
import org.nknsd.robotics.team.components.IntakeServoHandler;
import org.nknsd.robotics.team.components.PotentiometerHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.WheelDriver;
import org.nknsd.robotics.team.components.WheelHandler;
import org.nknsd.robotics.team.components.drivers.EACDriver;

import java.util.List;

public class BasicNKNProgram extends NKNProgram {
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
        telemetryEnabled.add(potentiometerHandler);


        // Arm
        RotationHandler rotationHandler = new RotationHandler ("motorArmRotate", 0.05, 0.38, 0.1, 1, false);
        components.add(rotationHandler);
        telemetryEnabled.add(rotationHandler);

        ExtensionHandler extensionHandler = new ExtensionHandler("motorArmExtend", true, 0.35);
        components.add(extensionHandler);
        //telemetryEnabled.add(extensionHandler);

        IntakeServoHandler intakeServoHandler = new IntakeServoHandler("intakeServo");
        components.add(intakeServoHandler);


        // Driver
        WheelDriver wheelDriver = new WheelDriver(0, 1, 5, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_Y, GamePadHandler.GamepadSticks.LEFT_JOYSTICK_X, GamePadHandler.GamepadSticks.RIGHT_JOYSTICK_X);
        components.add(wheelDriver);

        EACDriver eacDriver = new EACDriver();
        components.add(eacDriver);
        telemetryEnabled.add(eacDriver);


        // Link the components to each other
        wheelDriver.link(gamePadHandler, wheelHandler);
        rotationHandler.link(potentiometerHandler, extensionHandler);
        extensionHandler.link(rotationHandler);
        eacDriver.link(gamePadHandler, rotationHandler, extensionHandler, intakeServoHandler);
    }
}
