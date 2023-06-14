package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class OdoFriction extends HWMap {
    public Encoder encoder;
    public OdoFriction(Telemetry telemetry, HardwareMap hardwareMap){
        super(telemetry, hardwareMap);
        encoder = hardwareMap.get(Encoder.class, "encoder");
    }

    public void EndcoderTicks(){

        telemetry.addData("Encoder position:", encoder.getCurrentPosition());
        telemetry.addData("Encoder Velocity:", encoder.getCorrectedVelocity());
        telemetry.update();
    }
}
