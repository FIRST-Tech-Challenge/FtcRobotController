package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot {
        Telemetry telemetry;
        DriveTrain driveTrain;

        public Robot(Telemetry telemetry, HardwareMap hardwareMap)
        {
            this.telemetry=telemetry;
            driveTrain = new DriveTrain(telemetry, hardwareMap);

            motorInit();
        }
        public void motorInit()
        {
            driveTrain.motorInit();

        }
}
