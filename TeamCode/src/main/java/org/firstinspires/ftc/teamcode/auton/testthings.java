package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;

@Autonomous(name="bucket auton")
public class testthings extends LinearOpMode {
    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);//new thing

        //4
        telemetry.update();
        waitForStart();

        r.setLinearSlide(0.5, 20);
        r.setArm(0.5,20);
    }
}
