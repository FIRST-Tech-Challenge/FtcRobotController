package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
@TeleOp(name = "BackBradTeleop")

public class CS2Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this,true,false,true);
        while (!isStarted() || isStopRequested()) {
            robot.update();
        }
        while(opModeIsActive()&&!isStopRequested()){
            robot.teleOp();
        }
        robot.stop();
    }
}
