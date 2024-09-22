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
        FrontRightMotor = hardwareMap.get(DcMotorEx.class, "FRW");
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeftMotor = hardwareMap.get(DcMotorEx.class, "FLW");
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackRightMotor = hardwareMap.get(DcMotorEx.class, "BRW");
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeftMotor = hardwareMap.get(DcMotorEx.class, "BLW");
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        BackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        RightOdometerMotor = hardwareMap.get(DcMotorEx.class, "RIGHT_ODOMETER");
        RightOdometerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightOdometerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightOdometerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightOdometerMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftOdometerMotor = hardwareMap.get(DcMotorEx.class, "LEFT_ODOMETER");
        LeftOdometerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftOdometerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOdometerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftOdometerMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        CentreOdometerMotor = hardwareMap.get(DcMotorEx.class, "CENTRE_ODOMETER");
        CentreOdometerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CentreOdometerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CentreOdometerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CentreOdometerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Get all DcMotors associated with driving
     * @return Array of DcMotors, in order of FrontRight | FrontLeft | BackRight | BackLeft
     */
    public DcMotor[] GetDriveMotors() {
        return new DcMotor[] {FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor};

    }
}