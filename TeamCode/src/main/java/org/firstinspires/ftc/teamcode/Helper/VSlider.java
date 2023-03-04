package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VSlider {
    public DcMotor motor;

    private ElapsedTime runtime = new ElapsedTime();

    int timeout_ms;

    public int PositionForMedium = 4000;
    public int PositionForShort = 2000;
    public int PositionLowered = 0;


    /* local OpMode members. */
    //Init hardware map
    HardwareMap hwMap = null;


    //private static Linear Opmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        motor = hwMap.get(DcMotor.class, "vSlider");
        //Direction set in robot class
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void MoveSlider(double speed, int Position, int timeout) {
        timeout_ms = timeout;

        runtime.reset();


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(Position);
        //Set the power of the motor.
        motor.setPower(speed);
        //Run to position.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }
        motor.setPower(0);
    }

    public void MoveSliderWithEncoder (double speed, int Position) {
        timeout_ms = 5000;

        runtime.reset();
        motor.setTargetPosition(Position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        //Run to position.
        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }
        motor.setPower(0);

    }

}
