package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;

public class Lift {
    DcMotorEx liftMotor = null;
    PIDFController pidf = new PIDFController(0,0,0,0);

    private int level = 0;

    private double[] levels = {0,1000,2000,3000};

    private double power = 0;

    public Lift (HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
    }

    public void init(){
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoders();
    }

    public Action setPowerByPIDAndLevels(int level){
        this.level = level;
        if(liftMotor.getCurrent(CurrentUnit.AMPS)<=5 && level==0 && power<0){
            resetEncoders();
        }
        power = pidf.calculate(liftMotor.getCurrentPosition(),levels[level]);
        return new setPowerAction();
    }
    public void resetEncoders(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    public class setPowerAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            liftMotor.setPower(power);
            return true;
        }
    }
}
