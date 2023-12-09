package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous
public class RedAudience extends LinearOpMode {
    //Instantiate robot class
    RobotClass teamBot = new RobotClass(this);

    private int posOfTag;

    public void runOpMode() throws InterruptedException {
        //initialize robot

        teamBot.init(hardwareMap);
        posOfTag = teamBot.findTeamProp(502);

            waitForStart();

        if(posOfTag == 1){
            teamBot.moveWithoutEncoders(0.6,0.6,950);
            teamBot.strafing(RobotClass.Direction.LEFT,0.4,500);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.RIGHT,0.4,500);
            teamBot.moveWithoutEncoders(0.6,0.6,1700);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(0.8,0.8,4500);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            teamBot.strafing(RobotClass.Direction.RIGHT,0.4,500);


        }else if(posOfTag == 2){
            teamBot.moveWithoutEncoders(0.6, 0.6, 2500);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.LEFT, 0.6, 2000);
            teamBot.moveWithoutEncoders(0.6, 0.6, 1500);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(0.8,0.8,4800);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.4,500);

        }else if(posOfTag == 3){
            teamBot.moveWithoutEncoders(0.6, 0.6, 1000);
            teamBot.gyroTurning(90);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.5, 1000);
            teamBot.moveWithoutEncoders(-0.6, -0.6, 500);
            //Open Purple Pixel Claw Side
            //Shooting Purple Pixel onto SpikeMark with Intake
            teamBot.strafing(RobotClass.Direction.LEFT, 0.6, 1500);
            teamBot.moveWithoutEncoders(0.8,0.8,5200);
            //Open Yellow Pixel Claw Side
            //Shooting Yellow Pixel into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.4,500);

        }else{
            teamBot.moveWithoutEncoders(0.6,0.6,800);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.5, 1000);
            teamBot.moveWithoutEncoders(0.6,0.6,2200);
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.5, 1000);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(0.8,0.8,4800);
            //Open Purple and Yellow Pixel Claw Sides
            //Shooting Purple and Yellow Pixels into Backstage
            teamBot.moveWithoutEncoders(0.6,0.6,500);
            teamBot.strafing(RobotClass.Direction.RIGHT, 0.4,500);

        }
    }
}