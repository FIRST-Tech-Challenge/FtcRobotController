package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
@Config
@TeleOp(name = "LiftTest")

public class LiftTest extends LinearOpMode {
    public void runOpMode(){
        PwPRobot robot = new PwPRobot(this, true);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                robot.liftToPosition(Lift.LiftConstants.LIFT_GROUND_JUNCTION);
            }
            if(gamepad1.b){
                robot.liftToPosition(Lift.LiftConstants.LIFT_LOW_JUNCTION);
            }
            if(gamepad1.y){
                robot.liftToPosition(Lift.LiftConstants.LIFT_MED_JUNCTION);
            }
            if(gamepad1.x){
                robot.liftToPosition(Lift.LiftConstants.LIFT_HIGH_JUNCTION);
            }
            if(gamepad1.dpad_down){
                robot.liftToPosition(0);
            }
            if(gamepad1.dpad_right){
                robot.liftToPosition(200);
            }
            if(gamepad1.dpad_up){
                robot.liftToPosition(400);
            }
            if(gamepad1.dpad_left){
                robot.liftToPosition(600);
            }
            if(gamepad1.right_trigger!=0||gamepad1.left_trigger!=0){
                robot.setLiftPower(gamepad1.right_trigger-gamepad1.left_trigger);
            }else{
                robot.setLiftPower(0);
            }
            telemetry.update();
        }
    }
}
