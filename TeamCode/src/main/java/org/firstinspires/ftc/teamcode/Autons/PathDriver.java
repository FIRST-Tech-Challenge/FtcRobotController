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

		// X35.39382967329208 Y9.790647485576445 R204.1082421322776-O
		r.gyroTurn(204.1082421322776-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",204.1082421322776-r.imu.getHeading());

		r.AEncDrive(13.908677957757437,0,0.5,0,4000);
		r.gyroTurn(151.57057572909414-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",151.57057572909414-r.imu.getHeading());

		r.AEncDrive(6.506858730947473,0,0.5,0,4000);
		r.gyroTurn(125.19366133070609-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",125.19366133070609-r.imu.getHeading());

		r.AEncDrive(2.852488837462371,0,0.5,0,4000);
		r.gyroTurn(63.47077427380685-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",63.47077427380685-r.imu.getHeading());

		r.AEncDrive(3.1906836500166906,0,0.5,0,4000);
		r.gyroTurn(359.97467738627694-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",359.97467738627694-r.imu.getHeading());

		r.AEncDrive(2.9914087697246408,0,0.5,0,4000);
		r.gyroTurn(332.0386949341282-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",332.0386949341282-r.imu.getHeading());

		r.AEncDrive(1.9066502730986372,0,0.5,0,4000);
		r.gyroTurn(289.63633625664016-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",289.63633625664016-r.imu.getHeading());

		r.AEncDrive(17.54247463320425,0,0.5,0,4000);
		r.gyroTurn(231.78938024270388-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",231.78938024270388-r.imu.getHeading());

		r.AEncDrive(17.867514586078556,0,0.5,0,4000);
		r.gyroTurn(204.1082421322776-Math.abs(r.imu.getHeading()),0.5,4000);
		telemetry.addData("heading:",r.imu.getHeading());		
		telemetry.update();
		telemetry.addData("turn:",204.1082421322776-r.imu.getHeading());
	} 
}