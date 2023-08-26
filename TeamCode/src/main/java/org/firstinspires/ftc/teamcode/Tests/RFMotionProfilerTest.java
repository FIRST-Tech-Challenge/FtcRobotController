package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

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
        double minvelo = 2000;
        packet.clearLines();

        while (opModeIsActive()) {
            motor.setIsSim(true);
//
//            if (currentTickPos < 0 + 1/pow(10, 5)) {
//                motor.setTargetPos(1250);
//                motor.getTargetMotion(1);
//            }
//
//            else if (currentTickPos > 1250 - 1/pow(10, 5)) {
//                motor.setTargetPos(0);
//                motor.getTargetMotion(1);
//            }
//
////            Log.i("Time Intervals", Arrays.toString(motor.getTimeIntervals()));
//
////            Log.i("J", String.valueOf(motor.getJ()));
////
////            Log.i("Peak Velocity", String.valueOf(motor.getPeakVelo()));
////
////            Log.i("Starting Velocity", String.valueOf(motor.getStartingVelo()));
//
////            Log.i("Target Velocity", BasicRobot.time + " " + motor.getTargetVelocity(BasicRobot.time));
//
//            double power = motor.getTargetPower();
//            double velo = motor.getTargetVelocity(BasicRobot.time);
//            double pos = motor.getTargetPosition(BasicRobot.time);
//            double accel = motor.getTargetAcceleration(BasicRobot.time);
//
//            packet.put("Target Power", power);
//
//            packet.put("Target Velocity", velo);
//
//            packet.put("Target Acceleration", accel);
//
////            if (velo < minvelo) {
////                minvelo = velo;
////            }
////            packet.put("Min Velocity", minvelo);
//
//            packet.put("Target Position", pos);
//
//            packet.put("Tick Position", currentTickPos);
//
//            if (pos > maxpos) {
//                maxpos = pos;
//            }
//
//            packet.put("Max Position", maxpos);
//
////            if (BasicRobot.time >= motor.getTimeIntervals()[7]) {
////                resetRuntime();
////            }
//
//            motor.update();
//            robot.update();
        }
    }
}
