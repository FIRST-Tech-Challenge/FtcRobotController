package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.TimerTask;

public class PowerIncreaser extends TimerTask {
    HardwareInnov8Hera hera;
    Telemetry telemetry;
    LinearOpMode opMode;
    private double motorpower = 0.5;
    public PowerIncreaser(HardwareInnov8Hera hera, Telemetry telemetry, LinearOpMode opMode){
        this.hera = hera;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }
    @Override
    public void run() {
        hera.motorOne.setPower(motorpower);
        hera.motorTwo.setPower(motorpower);
        hera.motorThree.setPower(motorpower);
        hera.motorFour.setPower(motorpower);
        motorpower+=0.05;
    }
}
