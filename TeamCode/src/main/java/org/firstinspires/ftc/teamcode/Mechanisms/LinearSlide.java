package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide{
    Telemetry telemetry;
    public DcMotor LinearMotor;

    public LinearSlide(HardwareMap hardwareMap, String LinearMotorName, Telemetry telemetry ) {
        this.telemetry = telemetry;
        setup( hardwareMap, LinearMotorName );
    }

    public void setup( HardwareMap hardwareMap, String LinearMotorName ) {
        LinearMotor = hardwareMap.get(DcMotor.class,LinearMotorName);
    }

    public void setPower(double Power){
        LinearMotor.setPower(Power);
    }
}


