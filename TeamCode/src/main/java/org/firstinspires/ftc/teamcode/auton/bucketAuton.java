package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;

@Autonomous(name="bucket auton")
public class bucketAuton extends LinearOpMode {
    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);

        //4
        telemetry.update();
        waitForStart();

        r.moveInches(.5, 18, Hardware.directions.LEFT);
        r.moveInches(.5, 18);
    }
}

