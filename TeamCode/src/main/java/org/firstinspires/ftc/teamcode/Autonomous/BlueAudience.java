package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous()

public class BlueAudience extends LinearOpMode {

    RobotClass teamBot = new RobotClass(this);
    private int posOfTag;


    @Override
    public void runOpMode() throws InterruptedException {
        teamBot.init(hardwareMap);
        posOfTag = teamBot.findTeamProp(502);

        waitForStart();

        if(posOfTag == 1){
            teamBot.moveWithoutEncoders(0.6, 0.6, 1000);
            sleep(500);
            teamBot.gyroTurning(-90);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.5, 1000);
            sleep(500);
            teamBot.moveWithoutEncoders(-0.6, -0.6, 500);
            sleep(500);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.6, 1500);
            sleep(500);
            teamBot.moveWithoutEncoders(0.8,0.8,4800);
            sleep(500);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.4,500);
            sleep(500);

        }else if(posOfTag == 2){
            teamBot.moveWithoutEncoders(0.6, 0.6, 2500);
            sleep(500);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.6, 2000);
            sleep(500);
            teamBot.moveWithoutEncoders(0.6, 0.6, 1500);
            sleep(500);
            teamBot.gyroTurning(-90);
            sleep(500);
            teamBot.moveWithoutEncoders(0.8,0.8,4800);
            sleep(500);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.4,500);
            sleep(500);

        }else if(posOfTag == 3){
            teamBot.moveWithoutEncoders(0.6,0.6,950);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.RIGHT,0.4,500);
            sleep(500);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.RIGHT,0.4,500);
            sleep(500);
            teamBot.moveWithoutEncoders(0.6,0.6,1700);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.4,500);
            sleep(500);
            teamBot.gyroTurning(-90);
            sleep(500);
            teamBot.moveWithoutEncoders(0.8,0.8,4500);
            sleep(500);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.LEFT,0.4,500);
            sleep(500);

        }else{
            teamBot.moveWithoutEncoders(0.6,0.6,800);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.5, 1000);
            sleep(500);
            teamBot.moveWithoutEncoders(0.6,0.6,2200);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.5, 1000);
            sleep(500);
            teamBot.gyroTurning(-90);
            sleep(500);
            teamBot.moveWithoutEncoders(0.8,0.8,4800);
            sleep(500);
            //Open Purple and Yellow Pixel Claw Sides
            //Shooting Purple and Yellow Pixels into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            sleep(500);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.4,500);
            sleep(500);
        }
    }
}
