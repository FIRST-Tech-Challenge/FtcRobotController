package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPos;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.Arrays;

@Config
@Autonomous(name = "RFMotorDebugging")
public class RFMotorDebugging extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        RFMotor motor = new RFMotor("motorRightFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false, 10000, 0);
        waitForStart();

        if (isStopRequested()) return;

        robot.update();

        while (opModeIsActive()) {
            motor.setTargetPos(10000);
            motor.getTargetMotion(0.5);
//            Log.i("Time Intervals", Arrays.toString(motor.getTimeIntervals()));

//            Log.i("J", String.valueOf(motor.getJ()));
//
//            Log.i("Peak Velocity", String.valueOf(motor.getPeakVelo()));
//
//            Log.i("Starting Velocity", String.valueOf(motor.getStartingVelo()));

//            Log.i("Target Velocity", BasicRobot.time + " " + motor.getTargetVelocity(BasicRobot.time));

            packet.put("Target Velocity", motor.getTargetVelocity(BasicRobot.time));

            packet.put("Target Position", motor.getTargetPosition(BasicRobot.time));

            if (time >= motor.getTimeIntervals()[7]) {
                resetRuntime();
            }
            robot.update();
        }
    }
}
