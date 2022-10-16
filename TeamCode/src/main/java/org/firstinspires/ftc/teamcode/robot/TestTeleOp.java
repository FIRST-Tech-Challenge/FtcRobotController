package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: Test Scaffold", group="Robot")
public class TestTeleOp extends LinearOpMode {
    public BrainStemRobot robot;
    public final double SLOW = 0.5;
    public void runOpMode(){

        robot = new BrainStemRobot(hardwareMap, telemetry);
        robot.initializeRobotPosition();
        if(gamepad1.a){
            robot.moveTurret(90);
        }
        if(gamepad1.b){
            robot.moveTurret(180);
        }
        if(gamepad1.x){
            robot.moveTurret(0);
        }
        if(gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2){
            robot.lift.setMotor((gamepad1.left_stick_y * SLOW));
        }
        else{
            robot.lift.setMotor(0);
        }
        if(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2){
            robot.turret.setMotor((gamepad1.left_stick_x * SLOW));
        }
        else{
            robot.turret.setMotor(0);
        }
    }

}
