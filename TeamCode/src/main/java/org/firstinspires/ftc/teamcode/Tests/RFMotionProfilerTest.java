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
@Autonomous(name = "RFMotionProfilerTest")
public class RFMotionProfilerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        RFMotor motor = new RFMotor("motorRightFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false, 10000, 0);
        waitForStart();

        if (isStopRequested()) return;

        robot.update();

        double maxpos = 0;

        while (opModeIsActive()) {
            motor.setIsSim(true);
            motor.setTargetPos(2000);
            motor.getTargetMotion(1);
//            Log.i("Time Intervals", Arrays.toString(motor.getTimeIntervals()));

//            Log.i("J", String.valueOf(motor.getJ()));
//
//            Log.i("Peak Velocity", String.valueOf(motor.getPeakVelo()));
//
//            Log.i("Starting Velocity", String.valueOf(motor.getStartingVelo()));

//            Log.i("Target Velocity", BasicRobot.time + " " + motor.getTargetVelocity(BasicRobot.time));

            double power = motor.getTargetPower();

            packet.put("Target Power", power * 2000); //multiplied just to have same scale as velocity

            packet.put("Target Velocity", motor.getTargetVelocity(BasicRobot.time));

            packet.put("Target Position", motor.getTargetPosition(BasicRobot.time));

            if (motor.getTargetPosition(BasicRobot.time) > maxpos) {
                maxpos = motor.getTargetPosition(BasicRobot.time);
            }

            packet.put("Max Position", maxpos);

            if (time >= motor.getTimeIntervals()[7]) {
                resetRuntime();
            }

            motor.update();
            robot.update();
        }
    }
}
