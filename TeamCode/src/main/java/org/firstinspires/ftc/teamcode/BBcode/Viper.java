package org.firstinspires.ftc.teamcode.BBcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Viper {
    OpMode _opMode;
    DcMotorEx _viperMotor;
    DcMotorEx _armMotor;
    public Viper (OpMode opMode)
    {
        _opMode = opMode;
        _viperMotor = _opMode.hardwareMap.tryGet(DcMotorEx.class, "viperMotor");
        _armMotor = _opMode.hardwareMap.tryGet(DcMotorEx.class, "armMotor");
    }
    //--------------------------
    //Variable Storage:
    //RPM 312 = 537.7 Ticks per Revolution
    int fullExtend = 24;
    int halfExtend = 9;
    int specimenhangExtend = 7;
    int shortExtend = 3;
    //--------------------------

    public void ExtendFull(double power) {ViperMotorCustom(fullExtend, power);}
    public void ExtendShort(double power) {ViperMotorCustom(shortExtend, power);}
    public void ExtendHalf(double power) {ViperMotorCustom(halfExtend, power);}
    public void ExtendSpecimenhang(double power) {ViperMotorCustom(specimenhangExtend, power);}


    public void ViperMotorCustom(double lengthInches, double power)
    {
        if (_viperMotor == null)
        {
            _opMode.telemetry.addLine("Viper motor not found!");
            return;
        }
        //Full motor rotation = 7125 ticks
        //4 and 5/8 inches per rotation
        //~1541 ticks per inch

        //int ticksPerInch = 1541;
        double ticksPerInch = 537.7/4.625;
        //only need this if we have manual viper control
//        boolean minimumAngleTrue = _armMotor.getCurrentPosition() < 500;
//        if (minimumAngleTrue) {
//            lengthInches = Math.min(lengthInches, 18);
//        }
        _viperMotor.setDirection(DcMotor.Direction.REVERSE);
        int extensionTicks = (int)(lengthInches*ticksPerInch);
        _viperMotor.setTargetPosition(extensionTicks);    //Sets Target Tick Position
        _viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _viperMotor.setPower(power);

    }
}
