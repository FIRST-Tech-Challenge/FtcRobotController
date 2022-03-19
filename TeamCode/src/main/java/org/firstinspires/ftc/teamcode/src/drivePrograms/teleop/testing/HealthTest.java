package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

public class HealthTest extends GenericOpModeTemplate {

    CRServo spinnyServo;
    HardwareDeviceHealth h;
    ColorSensor s;

    @Override
    public void opModeMain() throws InterruptedException {
        spinnyServo = hardwareMap.crservo.get(GenericOpModeTemplate.carouselSpinnerName);
        h = (HardwareDeviceHealth) spinnyServo;
        s = hardwareMap.colorSensor.get(GenericOpModeTemplate.bucketColorSensorName);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            String info = spinnyServo.getConnectionInfo();
            HardwareDeviceHealth.HealthStatus healthStatus = h.getHealthStatus();
            telemetry.addData("Connection Info", info);
            telemetry.addData("Health", healthStatus.toString());

        }

    }
}
