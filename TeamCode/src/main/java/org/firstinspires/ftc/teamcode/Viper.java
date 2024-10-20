package org.firstinspires.ftc.teamcode;

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
    int fullExtend = 27;
    int halfExtend = 9;
    int shortExtend = 3;
    //--------------------------
    public void ExtendFull() {ViperMotorCustom(fullExtend);}
    public void ExtendShort() {ViperMotorCustom(shortExtend);}
    public void ExtendHalf() {ViperMotorCustom(halfExtend);}
    public void ViperMotorCustom(double lengthInches)
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
        boolean minimumAngleTrue = _armMotor.getCurrentPosition() < 500;
        if (minimumAngleTrue) {
            lengthInches = Math.min(lengthInches, 18);
        }
        _viperMotor.setDirection(DcMotor.Direction.REVERSE);
        int extensionTicks = (int)(lengthInches*ticksPerInch);
        _viperMotor.setTargetPosition(extensionTicks);    //Sets Target Tick Position
        _viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _viperMotor.setPower(0.3);

    }
}
