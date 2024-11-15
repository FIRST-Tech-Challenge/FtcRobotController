package org.firstinspires.ftc.teamcode.auton;

//import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;

@Autonomous(name="Specimen Auton")
public class specimenAuton extends LinearOpMode {
    MecanumHardAuto r = new MecanumHardAuto();

    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);

        telemetry.update();
        waitForStart();

        //specimen funny
        r.setWrist(0);
        r.waiter(500);
        r.setClaw(0);
        r.waiter(2000);
        r.moveInches(.5,24, Hardware.directions.LEFT);
        r.waiter(1000);
        r.moveInches(.5, 28);
        r.setWrist(0);
        r.setLinearSlide(.5,14);
        r.setArm(.5,80);
        r.linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        r.waiter(500);
        r.setLinearSlide(.5,7);
        r.waiter(200);
        r.setClaw(1);
        r.waiter(200);

        //park
        r.moveBackwards();
        r.moveInches(.5,26);
        r.moveForward();
        r.moveInches(.5,44, Hardware.directions.RIGHT);
        r.waiter(500);
        r.linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

    }
}

