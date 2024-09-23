package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */
public class TestingHardwareMap extends TeamHardwareMap {
    /*
        | FRW               | Front Right Wheel     | Expansion Hub Motor 0   |
        | FLW               | Front Left Wheel      | Control Hub Motor 0     |
        | BRW               | Back Right Wheel      | Expansion Hub Motor 1   |
        | BLW               | Back Left Wheel       | Expansion Hub Motor 1   |
        | RIGHT_ODOMETER    | Right Odometer        | Expansion Hub Motor 3   |
        | LEFT_ODOMETER     | Left Odometer         | Control Hub Motor 3     |
        | CENTRE_ODOMETER   | Centre Odometer       | Expansion Hub Motor 2   |
     */

    public DcMotorEx FrontRightMotor;
    public DcMotorEx FrontLeftMotor;
    public DcMotorEx BackRightMotor;
    public DcMotorEx BackLeftMotor;

    public DcMotorEx RightOdometerMotor;
    public DcMotorEx LeftOdometerMotor;
    public DcMotorEx CentreOdometerMotor;

    public TestingHardwareMap(HardwareMap hardwaremap) {
        super(hardwaremap);
    }

    /**
     * Sets up the following hardware bindings;
     * FrontRightMotor,
     * FrontLeftMotor,
     * BackRightMotor,
     * BackLeftMotor,
     * RightOdometerMotor,
     * LeftOdometerMotor,
     * CentreOdometerMotor
     */
    @Override
    protected void initialise() {
        ConfigureMovementMotor(FrontRightMotor, "FRW", DcMotorSimple.Direction.REVERSE);
        ConfigureMovementMotor(FrontLeftMotor, "FLW", DcMotorSimple.Direction.FORWARD);
        ConfigureMovementMotor(BackRightMotor, "BRW", DcMotorSimple.Direction.REVERSE);
        ConfigureMovementMotor(BackLeftMotor, "BLW", DcMotorSimple.Direction.FORWARD);

        ConfigureOdometerMotor(RightOdometerMotor, "RIGHT_ODOMETER");
        ConfigureOdometerMotor(LeftOdometerMotor, "LEFT_ODOMETER");
        ConfigureOdometerMotor(CentreOdometerMotor, "CENTRE_ODOMETER");
    }

    private void ConfigureMovementMotor(DcMotorEx motor, string name, DcMotorSimple.Direction dir) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        motor.setDirection(dir);
    }

    private void ConfigureOdometerMotor(DcMotorEx motor, string name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Get all DcMotors associated with driving
     * @return Array of DcMotors, in order of FrontRight | FrontLeft | BackRight | BackLeft
     */
    public DcMotor[] GetDriveMotors() {
        return new DcMotor[] {FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor};

    }
}