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
        -----------------------------------------------------------------------
        | FRW               | Front Right Wheel     | Control Hub Motor 0     |
        -----------------------------------------------------------------------
        | FLW               | Front Left Wheel      | Control Hub Motor 1     |
        -----------------------------------------------------------------------
        | BRW               | Back Right Wheel      | Control Hub Motor 2     |
        -----------------------------------------------------------------------
        | BLW               | Back Left Wheel       | Control Hub Motor 3     |
        -----------------------------------------------------------------------
        | RIGHT_ODOMETER    | Right Odometer        | None                    |
        -----------------------------------------------------------------------
        | LEFT_ODOMETER     | Left Odometer         | None                    |
        -----------------------------------------------------------------------
        | CENTRE_ODOMETER   | Centre Odometer       | None                    |
        -----------------------------------------------------------------------
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


    @Override
    protected void initialise() {
        // Setup and configure all drive motors
        ConfigureMovementMotor(FrontRightMotor, "FRW", DcMotorSimple.Direction.REVERSE);
        ConfigureMovementMotor(FrontLeftMotor, "FLW", DcMotorSimple.Direction.FORWARD);
        ConfigureMovementMotor(BackRightMotor, "BRW", DcMotorSimple.Direction.REVERSE);
        ConfigureMovementMotor(BackLeftMotor, "BLW", DcMotorSimple.Direction.FORWARD);

        // Odometers are not yet attached to robot
        // So dont setup to prevent errors

        // ConfigureOdometerMotor(RightOdometerMotor, "RIGHT_ODOMETER");
        // ConfigureOdometerMotor(LeftOdometerMotor, "LEFT_ODOMETER");
        // ConfigureOdometerMotor(CentreOdometerMotor, "CENTRE_ODOMETER");
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