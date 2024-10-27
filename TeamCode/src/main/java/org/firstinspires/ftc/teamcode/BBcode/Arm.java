package org.firstinspires.ftc.teamcode.BBcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    OpMode _opMode;
    DcMotorEx _armMotor;
    public Arm (OpMode opMode)
    {
        _opMode = opMode;
        _armMotor = _opMode.hardwareMap.tryGet(DcMotorEx.class, "armMotor");
    }
    //-----------------------
    //Variable Storage:
    //Total ticks in a revolution for 117 RPM motor: 1425.1
    int homePosition = 0;
    int clearancePosition = 24;
    int hangPosition = 65;
    int highBasketPosition = 80;
    int specimenPosition = 70;
    //-----------------------

    public boolean getIsHome()
    {
        return _armMotor.getCurrentPosition() < 10;
    }
    public void Stop()
    {
        _armMotor.setPower(0);
    }
    public void Reset()
    {
        if (_armMotor == null)
        {
            _opMode.telemetry.addLine("Arm motor not found!");
        } else {
            _armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            telemetryHelper.initMotorTelemetry( armMotor, "armMotor");
        }
    }
    public void MoveToHome()
    {
        ArmMotorCustom(homePosition);
    }
    public void MoveToClearance()
    {
        ArmMotorCustom(clearancePosition);
    }
    public void MoveToHang()
    {
        //target height is 21 inches
        //(entire front claw needs to be that height and clear robot front)
        ArmMotorCustom(hangPosition);
    }
    public void MoveToHighBasket()
    {
        ArmMotorCustom(highBasketPosition);
    }
    public void MoveToSpecimen()
    {
        //target height: 27 inches
        //(entire front claw needs to be that height and clear robot front)
        ArmMotorCustom(specimenPosition);
        //extension of viper slide to place specimens
    }
    public void ArmMotorCustom(int degrees)
    {
        if (_armMotor == null)
        {
            _opMode.telemetry.addLine("Arm motor not found!");
            return;
        }

        //Total ticks for armMotor drivetrain revolution: 7125
        //90 degree rotation for armMotor drivetrain revolution: 1781.25
        //int currentPosit = armMotor.getCurrentPosition();
        //targetArmDegrees = degrees;

        double ticksPerDegree = 7125.0/360.0;
        int convert = (int) (degrees*ticksPerDegree);
        //double difference = Math.abs(convert-currentPosit);
        _armMotor.setDirection(DcMotor.Direction.REVERSE);
        _armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _armMotor.setTargetPosition(convert);    //Sets Target Tick Position
        _armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _armMotor.setPower(0.2);
    }
}
