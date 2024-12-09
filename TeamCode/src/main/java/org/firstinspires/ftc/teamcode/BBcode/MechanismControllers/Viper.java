package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BBcode.TelemetryHelper;

public class Viper {
    OpMode _opMode;
    DcMotorEx _viperMotor;
    DcMotorEx _armMotor;
    public Viper (OpMode opMode)
    {
        TelemetryHelper telemetryHelper = new TelemetryHelper(opMode);
        _opMode = opMode;
        _viperMotor = _opMode.hardwareMap.tryGet(DcMotorEx.class, "viperMotor");
        telemetryHelper.initMotorTelemetry( _viperMotor, "VM");
        _armMotor = _opMode.hardwareMap.tryGet(DcMotorEx.class, "armMotor");
        telemetryHelper.initMotorTelemetry( _viperMotor, "VM");
    }
    //--------------------------
    //Variable Storage:
    //RPM 312 = 537.7 Ticks per Revolution
    int fullExtend = 24;
    int halfExtend = 9;
    int specimenhangExtend = 7;
    int shortExtend = 3;
    int closedExtend = 0;
    //--------------------------

    private int InchConverterToTicks(double lengthInches) {
        //Full motor rotation = 7125 ticks
        //4 and 5/8 inches per rotation
        //~1541 ticks per inch

        //int ticksPerInch = 1541;
        double ticksPerInch = 537.7/4.625;
        return (int)(lengthInches*ticksPerInch);
    }
    public DcMotorEx get_viperMotor() {
        return _viperMotor;
    }
    public boolean getIsViperExtendFull() {return _viperMotor.getCurrentPosition() > InchConverterToTicks(fullExtend);}
    public boolean getIsViperRetractedShort() {return _viperMotor.getCurrentPosition() < InchConverterToTicks(shortExtend + 0.5);}
    public boolean getIsViperExtendClosed() {return _viperMotor.getCurrentPosition() < InchConverterToTicks(closedExtend + 0.5);}

    public void StopAndResetEncoder() {_viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void ExtendFull(double power) {ViperMotorCustom(fullExtend, power);}
    public void ExtendShort(double power) {ViperMotorCustom(shortExtend, power);}
    public void ExtendHalf(double power) {ViperMotorCustom(halfExtend, power);}
    public void ExtendClosed(double power) {ViperMotorCustom(closedExtend, power);}
    public void ExtendSpecimenhang(double power) {ViperMotorCustom(specimenhangExtend, power);}
    public void ExtendClosed(double power) {ViperMotorCustom(closedExtend, power);}

    public void ViperMotorCustom(double lengthInches, double power)
    {
        if (_viperMotor == null)
        {
            _opMode.telemetry.addLine("Viper motor not found!");
            return;
        }

        //only need this if we have manual viper control
//        boolean minimumAngleTrue = _armMotor.getCurrentPosition() < 500;
//        if (minimumAngleTrue) {
//            lengthInches = Math.min(lengthInches, 18);
//        }
        _viperMotor.setDirection(DcMotor.Direction.REVERSE);
        int extensionTicks = InchConverterToTicks(lengthInches);
        _viperMotor.setTargetPosition(extensionTicks);    //Sets Target Tick Position
        _viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _viperMotor.setPower(power);

    }
}
