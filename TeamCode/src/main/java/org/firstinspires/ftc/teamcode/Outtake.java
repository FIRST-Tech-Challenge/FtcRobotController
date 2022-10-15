package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private DcMotor slide;

    Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        slide = hardwareMap.get(DcMotor.class, "slide");
    }

    public void run(double pow){
        slide.setPower(pow);
        slide.setPower(-pow);
    }
}
