package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.SampleDetector;

@Config
@TeleOp
public class SampleDetectorTestingOpMode extends OpMode {

    private Hardware hardware = new Hardware();

    private SampleDetector detector;

    @Override
    public void init() {
        hardware.init(hardwareMap);

        detector = new SampleDetector(hardware);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        detector.update();

        telemetry.addData("Distance: ", detector.getDistance());
        telemetry.addData("SampleDetected: ", detector.sampleDetected());
        telemetry.update();
    }
}
