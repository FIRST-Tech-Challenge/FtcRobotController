package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JarlsCHasse;


public class Jarlsnerg_odometry extends OpMode {

    @Override
    public void init() {

        JarlsCHasse driveTrain = new JarlsCHasse(hardwareMap);

    }

    @Override
    public void loop() {

    }
}
