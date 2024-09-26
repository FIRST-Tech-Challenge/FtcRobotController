package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */
public class OneMotorHardwareMap extends TeamHardwareMap {
    public DcMotorSimple FrontRightMotor;

    public OneMotorHardwareMap(HardwareMap hardwaremap) {
        super(hardwaremap);
    }

    @Override
    public void initialise() {
        ConfigureMovementMotor(FrontRightMotor, "FRW", DcMotorSimple.Direction.FORWARD);
    }

    private void ConfigureMovementMotor(DcMotorEx motor, String name, DcMotorSimple.Direction dir) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Necessary ?
        motor.setDirection(dir);
    }

    private void ConfigureOdometerMotor(DcMotorEx motor, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}