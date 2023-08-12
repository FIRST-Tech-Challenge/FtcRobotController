package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPos;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

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
        BasicRobot robot = new BasicRobot(this,true);
        RFMotor motor = new RFMotor("motorRightFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false, 10000, 0);
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();
        robot.update();
        BasicRobot.time = 0;
        currentPos = 0;
        currentVelocity = 0;
        boolean reversed = false;
        double targetPos = 9900;
        double maxPos = 0;
        double maxVelo = 0;
        double minPos = 10000000;
        double minVelo = 10000000;
        packet.put("curTickPos", currentPos);
        packet.put("curVelo", currentVelocity);
        packet.put("curPos", currentPos*0.036);
        packet.put("targetPos", targetPos);
        packet.put("maxPos", maxPos);
        packet.put("maxVelo", maxVelo);
        packet.put("minPos", minPos);
        packet.put("minVelo", minVelo);
        while (opModeIsActive()) {
            if (currentPos > maxPos) {
                maxPos = currentPos;
            }
            if (currentVelocity > maxVelo) {
                maxVelo = currentVelocity;
            }

            if (currentPos < minPos) {
                minPos = currentPos;
            }
            if (currentVelocity < minVelo) {
                minVelo = currentVelocity;
            }

            if (currentPos >= 9000 && !reversed) {
                targetPos = 100;
                reversed = true;
            }
//            else if (currentPos <= 1000 && reversed) {
//                targetPos = 9900;
//                reversed = false;
//            }

            motor.update(targetPos);

            robot.update();
            packet.put("curTickPos", currentPos);
            packet.put("curVelo", currentVelocity);
            packet.put("curPos", currentPos*0.036);
            packet.put("targetPos", targetPos);
            packet.put("maxPos", maxPos);
            packet.put("maxVelo", maxVelo);
            packet.put("minPos", minPos);
            packet.put("minVelo", minVelo);
        }
    }
}
