package org.firstinspires.ftc.teamcode.Autons;

//import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.driveUntilMechStop;
//import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.nEncDrive;
//import static org.firstinspires.ftc.teamcode.Disabled.CompBotV3.CompBotV3.runMotorTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

import java.util.Arrays;

@Autonomous(name="GUI Based Auton")
public class PathDriver extends LinearOpMode {
    public static final double dPower = 0.3;
    ElapsedTime runtime = new ElapsedTime();
    CompBotW1Attachments r = new CompBotW1Attachments();

    @Override
    public void runOpMode() {
        r.init(hardwareMap,true, telemetry,"red");
        // This below is the code strip

        // X19.928426758476544 Y20.905225899631912 R252.79479713144605-O
        r.gyroTurn(252.79479713144605-Math.abs(r.imu.getHeading()),0.5,4000);
        telemetry.addData("heading:",r.imu.getHeading());
        telemetry.update();
        telemetry.addData("turn:",252.79479713144605-r.imu.getHeading());

        r.AEncDrive(9.848163303095076,0,0.5,0,4000);
        r.gyroTurn(302.21305697141713-Math.abs(r.imu.getHeading()),0.5,4000);
        telemetry.addData("heading:",r.imu.getHeading());
        telemetry.update();
        telemetry.addData("turn:",302.21305697141713-r.imu.getHeading());

        r.AEncDrive(7.493650021472955,0,0.5,0,4000);
        r.gyroTurn(151.84360698428017-Math.abs(r.imu.getHeading()),0.5,4000);
        telemetry.addData("heading:",r.imu.getHeading());
        telemetry.update();
        telemetry.addData("turn:",151.84360698428017-r.imu.getHeading());

        r.AEncDrive(4.562076075258563,0,0.5,0,4000);
        r.gyroTurn(178.24268641556685-Math.abs(r.imu.getHeading()),0.5,4000);
        telemetry.addData("heading:",r.imu.getHeading());
        telemetry.update();
        telemetry.addData("turn:",178.24268641556685-r.imu.getHeading());

        r.AEncDrive(5.928052888868543,0,0.5,0,4000);
        r.gyroTurn(198.62021159159042-Math.abs(r.imu.getHeading()),0.5,4000);
        telemetry.addData("heading:",r.imu.getHeading());
        telemetry.update();
        telemetry.addData("turn:",198.62021159159042-r.imu.getHeading());

        r.AEncDrive(7.434563829179642,0,0.5,0,4000);
        r.gyroTurn(222.31083110781117-Math.abs(r.imu.getHeading()),0.5,4000);
        telemetry.addData("heading:",r.imu.getHeading());
        telemetry.update();
        telemetry.addData("turn:",222.31083110781117-r.imu.getHeading());

        r.AEncDrive(9.789666777307179,0,0.5,0,4000);
        r.gyroTurn(252.79479713144605-Math.abs(r.imu.getHeading()),0.5,4000);
        telemetry.addData("heading:",r.imu.getHeading());
        telemetry.update();
        telemetry.addData("turn:",252.79479713144605-r.imu.getHeading());
    }
}