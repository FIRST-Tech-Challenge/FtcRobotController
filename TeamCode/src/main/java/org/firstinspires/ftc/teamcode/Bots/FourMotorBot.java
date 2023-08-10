package org.firstinspires.ftc.teamcode.Bots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FourMotorBot {

    protected DcMotor leftFrontDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightBackDrive = null;

    protected Telemetry telemetry;


    public FourMotorBot(){}

    public void InitializeBot(HardwareMap hwMap,Telemetry telemetry)
    {
        this.telemetry = telemetry;
        getMotors(hwMap);
        setDirections();
    }

    // sets motors power.
    public void move(Gamepad gamepad)
    {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double drive = -gamepad.left_stick_y;
        double turn  =  gamepad.right_stick_x;

        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        rightFrontDrive.setPower(rightPower);

        telemetry.addData("Motors", "left (%.2f)", leftPower);
        telemetry.addData("Motors", "right (%.2f)", rightPower);
    }

    // Initialize Motors from hw data.
    private void getMotors(HardwareMap hwMap)
    {
        leftBackDrive = hwMap.get(DcMotor.class, Globals.BACK_MOTOR_L);
        leftFrontDrive = hwMap.get(DcMotor.class, Globals.FRONT_MOTOR_L);
        rightBackDrive = hwMap.get(DcMotor.class, Globals.BACK_MOTOR_R);
        rightFrontDrive = hwMap.get(DcMotor.class, Globals.BACK_MOTOR_R);
    }

    // Set direction for each motor.
    private void setDirections()
    {
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    }



}
