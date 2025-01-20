package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * FrontTouch is a subsystem that manages the front touch sensor of the robot.
 * It extends SubsystemBase and overrides the periodic method to check the sensor state.
 */
public class FrontTouch extends SubsystemBase {

    // Local objects and variables here

    /** The touch sensor object */
    private final TouchSensor touchSensor;

    /** The distance measured by the front sensor */
    private static double frontDistance;

    /**
     * Constructor for the FrontTouch subsystem.
     * Initializes the touch sensor using the hardware map.
     */
    public FrontTouch() {
        touchSensor = RobotContainer.ActiveOpMode.hardwareMap.get(TouchSensor.class, "frontTouch");
    }

    /**
     * Method called periodically by the scheduler.
     * This method checks if the touch sensor is pressed.
     */
    @Override
    public void periodic() {
        hasTouched();
    }

    /**
     * Checks if the touch sensor is pressed.
     *
     * @return true if the touch sensor is pressed, false otherwise.
     */
    public boolean hasTouched() {
        return touchSensor.isPressed();
    }
}