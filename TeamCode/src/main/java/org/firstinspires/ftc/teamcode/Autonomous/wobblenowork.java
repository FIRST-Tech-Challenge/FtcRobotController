package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "wobblenowork", preselectTeleOp = "TwoGPTeleop")

public class wobblenowork extends LinearOpMode {
    @Override
    public void runOpMode(){
        WobbleGoal robot = new WobbleGoal(this, false);
        waitForStart();
        //ok epic xXgamerXx heres how the game works
        //this line will slightly move up the wobble goal arm
        robot.wobbleGoalMotor.setPower(0.2);
        sleep(200);
        //stup
        robot.wobbleGoalMotor.setPower(0);
        sleep(1000);
        //this line will slightly move it down
        robot.wobbleGoalMotor.setPower(-0.2);
        sleep(200);
        //stup
        robot.wobbleGoalMotor.setPower(0);
        sleep(1000);
        //this line will move it tograb(orthis|    | position)
        robot.goToPosition(WobbleGoal.Position.GRAB);
        //stup
        sleep(5000);
        //this line doesn't work because wobble goal motor is still in RUN_TO_POSITION mode and no target has been set up
        robot.wobbleGoalMotor.setPower(0.2);
        sleep(200);
        //still stup
        robot.wobbleGoalMotor.setPower(0);
        sleep(1000);
        //useless
        robot.wobbleGoalMotor.setPower(-0.2);
        sleep(200);
        //still stup again
        robot.wobbleGoalMotor.setPower(0);
        sleep(1000);
        //this line will make the wobble goal motor go back to start position
        robot.goToPosition(WobbleGoal.Position.REST);
        //stop
        sleep(5000);
        //stop
        stop();
        //ok gamer stop gaming
    }
}

