package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
@TeleOp
public class ResetHanger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this,true);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            robot.hangerTele();
        }
        robot.stop();
    }
}
