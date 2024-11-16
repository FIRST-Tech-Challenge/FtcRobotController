package org.firstinspires.ftc.teamcode.Mechanisms.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

public class Robot {
    public Drivetrain drivetrain;
    public Battery battery;
    public Robot(HardwareMap hardwareMap){
        this.battery = new Battery(hardwareMap);
        this.drivetrain = new Drivetrain(hardwareMap,battery);
    }
}
