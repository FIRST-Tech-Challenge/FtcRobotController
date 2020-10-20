package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    String DRIVE_TRAIN_CAPTION = "Drive Status";
    Telemetry telemetry;
    HardwareInnov8Robot robot;
    LinearOpMode opMode;

    public Shooter(Telemetry telemetry, HardwareInnov8Robot robot, LinearOpMode opMode) {

        this.opMode = opMode;
        this.robot = robot;
        this.telemetry = telemetry;
    }
    
    public void teleop(Gamepad gamepad1, Gamepad gamepad2) {


    }

    public void shoot() {


    }

}
