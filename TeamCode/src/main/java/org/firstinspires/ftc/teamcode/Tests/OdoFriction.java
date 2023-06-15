package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.RR.util.Encoder;

public class OdoFriction {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public Encoder encoder;
    public OdoFriction(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        encoder = hardwareMap.get(Encoder.class, "encoder");
    }

    public void EndcoderTicks(){

        telemetry.addData("Encoder position:", encoder.getCurrentPosition());
        telemetry.addData("Encoder Velocity:", encoder.getCorrectedVelocity());
        telemetry.update();
    }
}
