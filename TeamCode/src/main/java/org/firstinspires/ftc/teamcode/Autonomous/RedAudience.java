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

    private Object posOfTag;

    public void runOpMode() throws InterruptedException {
        //initialize robot

        teamBot.init(hardwareMap);
        posOfTag = teamBot.findTeamProp(502);

            waitForStart();

        if (posOfTag == 1){


        }else if(posOfTag == 2){


        }else if(posOfTag == 3){


        }else{
            //ADD CODE HERE
        }

        //Moving to spike mark grid square
        teamBot.moveWithoutEncoders(0.6, 0.6, 2500);
        //Move forwards one more square
        teamBot.moveWithoutEncoders(0.6, 0.6, 1500);
        //Turning 90 degrees
        teamBot.gyroTurning(90);
        //Move forwards to the wall
        teamBot.moveWithoutEncoders(0.6, 0.6, 6000);
        //Strafe for Safety
        teamBot.strafing(RobotClass.Direction.RIGHT, 0.5, 1000);
    }
}