package org.firstinspires.ftc.teamcode.JackBurr.Camera.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JackBurr.Other.Range;

@TeleOp
public class GetNeededCameraAngleOfRotation extends OpMode {
    public LimelightV1 limelight = new LimelightV1();
    public Range center = new Range(0, 2);
    @Override
    public void init() {
        limelight.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

    }
}
