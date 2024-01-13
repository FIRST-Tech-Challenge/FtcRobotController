package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous()

public class BlueAudience extends LinearOpMode {

    RobotClass teamBot = new RobotClass(this);
    private int posOfTag;


    @Override
    public void runOpMode() throws InterruptedException {
        teamBot.init(hardwareMap);
//        posOfTag = teamBot.findTeamProp(502);

        waitForStart();

        if(posOfTag == 1){
            teamBot.moveStraightWithEncoders(0.6, 30);
            teamBot.gyroTurning(-90);
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.5, 1000);
            teamBot.moveStraightWithEncoders(-0.6,10);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.6, 1500);
            teamBot.moveStraightWithEncoders(0.8,220);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveStraightWithEncoders(0.6,10);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.4,500);

        }else if(posOfTag == 2){
            teamBot.moveStraightWithEncoders(0.6, 46);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.6, 2000);
            teamBot.moveStraightWithEncoders(0.6, 26);
            teamBot.gyroTurning(-90);
            teamBot.moveStraightWithEncoders(0.8,200);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveStraightWithEncoders(0.6,10);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.4,500);

        }else if(posOfTag == 3){
            teamBot.moveStraightWithEncoders(0.6,26);
            teamBot.strafing(RobotClass.Direction.RIGHT,0.4,500);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.RIGHT,0.4,500);
            teamBot.moveStraightWithEncoders(0.6,100);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.4,500);
            teamBot.gyroTurning(-90);
            teamBot.moveStraightWithEncoders(0.8,200);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveStraightWithEncoders(0.6,10);
            teamBot.strafing(RobotClass.Direction.LEFT,0.4,500);

        }else{
//            teamBot.moveStraightWithEncoders(0.4,130);
//            teamBot.gyroTurning(90);
//            teamBot.moveStraightWithEncoders(0.5,215);
//            teamBot.strafing(RobotClass.Direction.LEFT,0.2,400);
            teamBot.moveStraightWithEncoders(0.6,-122);
            //Shoot Purple Pixel
            teamBot.gyroTurning(90);
            teamBot.moveStraightWithEncoders(0.6, -244);

        }
    }
}
