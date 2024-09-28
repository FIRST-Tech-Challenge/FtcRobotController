package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */
public class DeepHardwareMap {
    /*
        -----------------------------------------------------------------------
        | FRW               | Front Right Wheel     | Control Hub Motor 3     |
        --------------------+-----------------------+--------------------------
        | FLW               | Front Left Wheel      | Control Hub Motor 2     |
        --------------------+-----------------------+--------------------------
        | BRW               | Back Right Wheel      | Control Hub Motor 0     |
        --------------------+-----------------------+--------------------------
        | BLW               | Back Left Wheel       | Control Hub Motor 1     |
        --------------------+-----------------------+--------------------------
        | RIGHT_ODOMETER    | Right Odometer        | None                    |
        --------------------+-----------------------+--------------------------
        | LEFT_ODOMETER     | Left Odometer         | None                    |
        --------------------+-----------------------+--------------------------
        | CENTRE_ODOMETER   | Centre Odometer       | None                    |
        -----------------------------------------------------------------------
     */

    private final HardwareMap hardwareMap;

    // Setup and configure all drive motors
    public DcMotorSimple FrontRightMotor = ConfigureMovementMotor("FRW", DcMotorSimple.Direction.REVERSE);
    public DcMotorSimple FrontLeftMotor = ConfigureMovementMotor("FLW", DcMotorSimple.Direction.REVERSE);
    public DcMotorSimple BackRightMotor = ConfigureMovementMotor("BRW", DcMotorSimple.Direction.REVERSE);
    public DcMotorSimple BackLeftMotor = ConfigureMovementMotor("BLW", DcMotorSimple.Direction.REVERSE);

    // Setup and configure all odometers
    // public DcMotorEx RightOdometerMotor = ConfigureOdometerMotor("RIGHT_ODOMETER");
    // public DcMotorEx LeftOdometerMotor = ConfigureOdometerMotor("LEFT_ODOMETER");
    // public DcMotorEx CentreOdometerMotor = ConfigureOdometerMotor("CENTRE_ODOMETER");

    public DeepHardwareMap(HardwareMap hardwaremap) {
        hardwareMap = hardwaremap;
        initialise();
    }

    public void initialise() {
        // Any additional configuration options here
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private DcMotorSimple ConfigureMovementMotor(String name, DcMotorSimple.Direction dir) {
        DcMotorSimple motor = hardwareMap.get(DcMotorSimple.class, name);
        motor.setDirection(dir);
        return motor;
    }

    private DcMotorEx ConfigureOdometerMotor(String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        return motor;
    }
}
