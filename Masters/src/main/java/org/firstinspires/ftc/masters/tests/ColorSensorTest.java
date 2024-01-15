package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class ColorSensorTest extends LinearOpMode {

    RevColorSensorV3 colorSensor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Sensed Red", colorSensor.red());
            telemetry.addData("Sensed Blue", colorSensor.blue());
            telemetry.addData("Sensed Green", colorSensor.green());
            telemetry.addData("Distance CM", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Light Detected", colorSensor.getLightDetected());
            telemetry.addData("Raw Light Detected", colorSensor.getRawLightDetected());
            telemetry.addData("Max Raw Light Detected", colorSensor.getRawLightDetectedMax());
            telemetry.addData("Raw Optics", colorSensor.rawOptical());
            telemetry.addData("Status", colorSensor.status());
            telemetry.update();

            if (colorSensor.getRawLightDetected() > CSCons.pixelDetectThreshold){
                clawServo.setPosition(0.82);
            } else {
                clawServo.setPosition(0.4);
            }
        }
    }
}
