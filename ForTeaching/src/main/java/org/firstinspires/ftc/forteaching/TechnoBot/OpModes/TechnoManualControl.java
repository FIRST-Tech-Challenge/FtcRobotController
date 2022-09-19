package org.firstinspires.ftc.forteaching.TechnoBot.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.util.Alliance;

import org.firstinspires.ftc.forteaching.TechnoBot.Controls;
import org.firstinspires.ftc.forteaching.TechnoBot.Hardware;
import org.firstinspires.ftc.forteaching.TechnoBot.TheBot;

@TeleOp(name = "Tank Driving", group = "TechnoLib")
public class TechnoManualControl extends CommandOpMode {
    public Hardware hardware;
    public TheBot robot;
    public Controls controls;

    @Override
    public void uponInit() {
        hardware = new Hardware(hardwareMap);
        robot = new TheBot(hardware);
        controls = new Controls(driverGamepad, robot, Alliance.RED);
    }
}
