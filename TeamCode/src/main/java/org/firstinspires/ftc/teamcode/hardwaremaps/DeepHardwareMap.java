package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */
public class DeepHardwareMap extends TeamHardwareMap {
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

    public DcMotorSimple FrontRightMotor;
    public DcMotorSimple FrontLeftMotor;
    public DcMotorSimple BackRightMotor;
    public DcMotorSimple BackLeftMotor;

    public DcMotorEx RightOdometerMotor;
    public DcMotorEx LeftOdometerMotor;
    public DcMotorEx CentreOdometerMotor;

    public DeepHardwareMap(HardwareMap hardwaremap) {
        super(hardwaremap);
    }


    @Override
    public void initialise() {
        // FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        // Setup and configure all drive motors
        // ConfigureMovementMotor(FrontRightMotor, "FRW", DcMotorSimple.Direction.REVERSE);
        FrontRightMotor = ConfigureMovementMotor(FrontLeftMotor, "FRW", DcMotorSimple.Direction.FORWARD);
        FrontLeftMotor = ConfigureMovementMotor(FrontLeftMotor, "FLW", DcMotorSimple.Direction.REVERSE);
        BackRightMotor = ConfigureMovementMotor(BackRightMotor, "BRW", DcMotorSimple.Direction.REVERE);
        BackLeftMotor = ConfigureMovementMotor(BackLeftMotor, "BLW", DcMotorSimple.Direction.REVERSE);


        // Odometers are not yet attached to robot
        // So dont setup to prevent errors

        // ConfigureOdometerMotor(RightOdometerMotor, "RIGHT_ODOMETER");
        // ConfigureOdometerMotor(LeftOdometerMotor, "LEFT_ODOMETER");
        // ConfigureOdometerMotor(CentreOdometerMotor, "CENTRE_ODOMETER");
    }

    private DcMotorSimple ConfigureMovementMotor(DcMotorSimple motor, String name, DcMotorSimple.Direction dir) {
        motor = hardwareMap.get(DcMotorSimple.class, name);
        // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        motor.setDirection(dir);
        return motor;
    }

    private void ConfigureOdometerMotor(DcMotorEx motor, String name) {
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
    public DcMotorSimple[] GetDriveMotors() {
        return new DcMotorSimple[] {FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor};

    }
}