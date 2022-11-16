package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift{
    Telemetry telemetry;
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry ) {
        this.telemetry = telemetry;
        setup( hardwareMap );
    }

    public void setup( HardwareMap hardwareMap ) {
        liftMotor1 = hardwareMap.get(DcMotor.class,"Test1");
        liftMotor2 = hardwareMap.get(DcMotor.class,"Test2");

    }

    public void SetMotorPower(double Power){
        liftMotor1.setPower(Power);
        liftMotor2.setPower(-Power);
    }

    public void driveLiftToPosition(double power, int armPosition) {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setTargetPosition(-armPosition);
        liftMotor2.setTargetPosition(armPosition);

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);

        while(liftMotor1.isBusy()&&liftMotor2.isBusy()){
        }

    }
}


