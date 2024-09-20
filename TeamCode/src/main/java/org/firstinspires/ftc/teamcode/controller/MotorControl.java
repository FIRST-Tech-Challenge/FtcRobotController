package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Logging;

public class MotorControl
{
    private DcMotor motor;
    private String motorName;
    private boolean hasEncoder;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    //Will be used to get the parameters below from the masterclass
    private MotorControl(OpMode opMode)
    {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
    }

    /**
     * @param motorName Name that you well enter for configuration
     * @param direction Direction you want the motor to spin: true = FORWARD, false = REVERSE
     * @param hasEncoder Does it have an encoder?
     */
    protected MotorControl(String motorName, Boolean direction, Boolean hasEncoder, OpMode opMode)
    {
        this(opMode);

        this.motorName = motorName;
        this.hasEncoder = hasEncoder;
        try
        {
            motor = this.hardwareMap.get(DcMotor.class, motorName);

            if (direction) {motor.setDirection(DcMotorSimple.Direction.FORWARD);}
            else {motor.setDirection(DcMotorSimple.Direction.REVERSE);}

            if (hasEncoder) {motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        catch (IllegalArgumentException e)
        {
            Logging.log("%s not found in Hardware Map",motorName);
            telemetry.addData("Exception:", "%s not found in Hardware Map",motorName);
            telemetry.update();

            //sleep(1000);

        }

    }

    /**
     * Analog control method without bounds
     *
     * @param speedLimit Put's restriction on how fast the motor can spin
     * @param input which gamepad float value that will mak this spin
     */
    protected void analogControl(double speedLimit, double input, boolean advanceBreak)
    {
        double motorPower = input;
        motorPower = Range.clip(motorPower, -speedLimit, speedLimit);

        if (Math.abs(motorPower) > 0)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(motorPower);
        }
        else if (advanceBreak && motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setPower(0.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (advanceBreak && motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {}
        else {motorBreak();}

    }

    /**
     * Analog control method with bounds
     *
     * @param speedLimit Put's restriction on how fast the motor can spin
     * @param input which gamepad float value that will mak this spin
     * @param lowerBound Motor will not spin past this bound at negative power (must have encoder to use this feature)
     * @param upperBound Motor will not spin past this bound at positive power(must have encoder to use this feature)
     */
    protected void analogControl(double speedLimit, double input, boolean advanceBreak, int lowerBound, int upperBound)
    {
        double motorPower = input;
        motorPower = Range.clip(motorPower, -speedLimit, speedLimit);

        if (Math.abs(motorPower) > 0)
        {
            if (100 + Math.abs(motor.getCurrentPosition()) > 100 + Math.abs(upperBound) && motorPower > 0) {telemetry.addData("Upper bound break", "");motorBreak();}
            else if (100 + Math.abs(motor.getCurrentPosition()) < 100 + Math.abs(lowerBound) && motorPower < 0) {telemetry.addData("Lower bound break", "");motorBreak();}
            else
            {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(motorPower);
            }
        }
        else if (advanceBreak && motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setPower(0.3);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (advanceBreak && motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {}
        else {motorBreak();}

    }

    /**
     * @param speed The speed at which the motor will spin
     * @param argument1 The Gamepad bool input that will make it spin forward
     * @param argument2 The Gamepad bool input that will make it spin backwards
     */
    protected void simpleDrive(double speed, boolean argument1, boolean argument2)
    {
        if (argument1)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            run(speed);
        }
        else if (argument2)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            run(-speed);
        }
        else {motorBreak();}
    }

    /**
     * @param speed The speed at which the motor will spin
     * @param argument The Gamepad bool input that will make it spin forward
     */
    protected void simpleDrive(double speed, boolean argument)
    {
        if (argument)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            run(speed);
        }
        else {motorBreak();}
    }

    /**
     * @param target Target location that the motor will move to
     * @param speed The speed at which the motor will spin
     * @param argument The Gamepad bool input that will make it move
     */
    public void encoderControl(int target, double speed, boolean argument)
    {
        if (argument)
        {
            encoderControl(target, speed);
        }
    }

    /**
     * @param target Target location that the motor will move to
     * @param speed The speed at which the motor will spin
     * @param argument The Gamepad analog input that will make it move if its value is greater than 0.2
     */
    public void encoderControl(int target, double speed, double argument)
    {
        if (Math.abs(argument) > 0.2)
        {
            encoderControl(target, speed);
        }
    }

    /**
     * @param target Target location that the motor will move to
     * @param speed The speed at which the motor will spin
     */
    public void encoderControl(int target, double speed)
    {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
    }

    /**
     * for motors that just need to spin call break to stop
     * @param speed speed you want the motor to spin
     */
    protected void run(double speed)
    {
        motor.setPower(speed);
    }


    /**
     *     Breaking method also sets power to zero
     */
    protected void motorBreak()
    {
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData(motorName, "Breaking");
    }

    /**
     *     Prints motor telemetry
     */
    protected void telemetry()
    {
        if (hasEncoder) {telemetry.addData(motorName, "Speed: %.2f\n\tEncoder Position: %d", motor.getPower(), motor.getCurrentPosition());}
        else {telemetry.addData(motorName, "Speed: %.2f", motor.getPower());}
    }

    protected int getMotorPosition()
    {
        return motor.getCurrentPosition();
    }

    protected boolean isBusy()
    {
        return motor.isBusy();
    }
}
