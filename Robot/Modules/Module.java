package org.firstinspires.ftc.teamcode.rework.Robot.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Module {
    HardwareMap hardwareMap;

    /**
     * Do not use this constructor. Does not assign hardwareMap or initialize the module.
     */
    public Module() {}

    /**
     * Initializes the module. This includes setting up all motors/servos
     * */
    abstract public void init();

    /**
     * Updates the module, executing all the tasks it should complete on every iteration,
     * utilizing the module's states. This separate method allows for each module to be updated
     * on a different thread from where the states are set.
     */
    abstract public void update();

    /**
     * Gets a DcMotor from the hardwareMap given the name of the motor, returning null if the
     * motor does not exist.
     *
     * @param name of the DcMotor to return.
     * @return DcMotor from hardwareMap, or null if the motor does not exist.
     */
    protected DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Motor could not be found. Exception: " + exception);
        }
    }
}
