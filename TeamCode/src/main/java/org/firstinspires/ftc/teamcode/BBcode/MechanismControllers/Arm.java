package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BBcode.TelemetryHelper;

public class Arm {
    OpMode _opMode;
    DcMotorEx _armMotor;
    TelemetryHelper _telemetryHelper;

    public Arm (OpMode opMode, TelemetryHelper telemetryHelper)
    {
        _opMode = opMode;
        _telemetryHelper = telemetryHelper;
        _armMotor = _opMode.hardwareMap.tryGet(DcMotorEx.class, "armMotor");
        _telemetryHelper.initMotorTelemetry( _armMotor, "AM");

    }
    //-----------------------
    //Variable Storage:
    //Total ticks in a revolution for 117 RPM motor: 1425.1
    //variables are in degrees
    final int homePosition = 0;
    final int slowDownPosition = 5;
    final int hangPosition = 40;
    final int highBasketPosition = 95;
    final double initialSpecimenPosition = 42.5;//used in auto and tele
    final double maxSpecimenPosition = 49;
    final double minSpecimenPosition = 39;
    final double specimanAdjustment = 0.5;
    public double specimenPosition = initialSpecimenPosition;
    //-----------------------

    private int DegreeConverterToTicks(double degrees) {
        //Total ticks for armMotor drivetrain revolution: 7125
        //90 degree rotation for armMotor drivetrain revolution: 1781.25
        //int currentPosit = armMotor.getCurrentPosition();
        //targetArmDegrees = degrees;

        double ticksPerDegree = 7125.0/360.0;
        return (int)(degrees*ticksPerDegree);
    }
    public boolean getIsHome()
    {
        return _armMotor.getCurrentPosition() < 10;
    }

    public boolean getIsArmHighBasketPosition() {return _armMotor.getCurrentPosition() > DegreeConverterToTicks(highBasketPosition);}
    public boolean getIsArmHomePosition() {return _armMotor.getCurrentPosition() < DegreeConverterToTicks(homePosition + 5);}
    public boolean getIsArmSpecimenPosition() {return _armMotor.getCurrentPosition() > DegreeConverterToTicks(specimenPosition);}
    public boolean getIsArmHangPosition() {return _armMotor.getCurrentPosition() < DegreeConverterToTicks(hangPosition);}
    public boolean getIsArmSlowDownPosition() {return _armMotor.getCurrentPosition() < DegreeConverterToTicks(slowDownPosition);}

    public DcMotorEx get_armMotor() {
        return _armMotor;
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
    public void IncreaseSpecimenAngle()
    {
        specimenPosition += specimanAdjustment;
        specimenPosition = Math.min(maxSpecimenPosition, specimenPosition);
    }
    public void DecreaseSpecimenAngle()
    {
        specimenPosition -= specimanAdjustment;
        specimenPosition = Math.max(minSpecimenPosition, specimenPosition);
    }
    public void MoveToHome()
    {
        ArmMotorCustom(homePosition, 0.25);
    }
    public void MoveToSlowDown()
    {
        ArmMotorCustom(slowDownPosition, 0.75);
    }
    public void MoveToHang()
    {
        ArmMotorCustom(hangPosition, 1);
    }
    public void MoveToHighBasket()
    {
        ArmMotorCustom(highBasketPosition, 1);
    }
    public void MoveToSpecimen()
    {
        MoveToSpecimen(1);
    }
    public void MoveToSpecimen(double power)
    {
        //target height: 27 inches
        //(entire front claw needs to be that height and clear robot front)
        ArmMotorCustom(specimenPosition, power);
        //extension of viper slide to place specimens
    }

    public void Rest() {_armMotor.setPower(0);}

    public void ArmMotorCustom(double degrees, double power)
    {
        if (_armMotor == null)
        {
            _opMode.telemetry.addLine("Arm motor not found!");
            return;
        }

        int convert = DegreeConverterToTicks(degrees);
        //double difference = Math.abs(convert-currentPosit);
        _armMotor.setDirection(DcMotor.Direction.REVERSE);
        _armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _armMotor.setTargetPosition(convert);    //Sets Target Tick Position
        _armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _armMotor.setPower(power);
    }
}
