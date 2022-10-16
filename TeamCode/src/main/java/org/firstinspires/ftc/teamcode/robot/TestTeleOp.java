package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: Test Scaffold", group="Robot")
public class TestTeleOp extends LinearOpMode {
    public BrainStemRobot robot;
    public void runOpMode(){
        robot = new BrainStemRobot(hardwareMap, telemetry);
        robot.initializeRobotPosition();
        if(gamepad1.a){
            robot.moveTurret(90);
        }
        if(gamepad1.b){
            robot.moveTurret(180);
        }
    }
}
