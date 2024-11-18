package org.firstinspires.ftc.teamcode.auton;

//import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;

//@Config
@Autonomous(name="Simple Park Auton")
public class autonTemplate extends LinearOpMode {
    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);



 //4
        telemetry.update();
        waitForStart();



    }
}
