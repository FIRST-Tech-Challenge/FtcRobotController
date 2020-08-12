package org.firstinspires.ftc.teamcode.rework.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareGlobals {

    protected static HardwareMap hardwareMap;

    /**
     * Gets a DcMotor from the hardwareMap given the name of the motor, returning null if the
     * motor does not exist.
     *
     * @param name of the DcMotor to return.
     * @return DcMotor from hardwareMap, or null if the motor does not exist.
     */
    public static DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Motor with name " + name + " could not be found. Exception: " + exception);
        }
    }


}
