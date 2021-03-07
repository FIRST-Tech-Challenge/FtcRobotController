package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "FinalAuto", preselectTeleOp = "OneGPTeleop")

public class FinalAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        if(rings!=1&&rings!=4) {
            robot.goToPosition( -66,-6,0, 1);//-25,60
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-61.5,33.8,-2,0.8);//37,4
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-53.5+43,32+6,0,0.8);
            robot.turnInPlace(0,0.6);
            robot.goToPosition(-53.5+43.5,32+10.5-5.8-3,0,0.8);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-53.5+43.5-56.5-7,32+3-6.5-24,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-53.5+43.5-59-5,32+6.5-4-23+6,0,0.6);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.turnInPlace(180,0.7);
        }
        else if(rings==1) {
            robot.goToPosition(-37,-8.5,0,1);
            robot.goToPosition(-96,14,0,1);
            robot.openWobbleGoalClaw();
            sleep(200);
            robot.goToPosition(-56.5,28.0,-2.5,0.7);//yPosition - 23
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-53.5+44,36,0,1);//yPosition - 26
            robot.turnInPlace(0,1.0);
            robot.goToPosition(-53.5+43,27,0,0.8);
            sleep(200);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-88,18,0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-72,23,0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);
            robot.turnInPlace(180,0.7);
        }
         else if(rings==4) {
            robot.goToPosition(-23,-10,-13.5,0.9);
            robot.shootHighGoalTest(1650, 100,3);
            robot.goToPosition(-115, -7, 0,1);
            robot.openWobbleGoalClaw();
            robot.goToPosition(-60,-2,1,1);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-45,-1,0.5,0.7);
            robot.goToPosition(-43,0,-2,0.4);
            sleep(500);
            robot.shootHighGoal(1);
//            robot.startIntake();
//            robot.startTransfer();
            sleep(250);
            robot.goToPosition(-33,0,0,1);
            robot.goToPosition(-33,0,-2,1);
            robot.shootHighGoal(3);
            robot.stopIntake();
            robot.stopTransfer();
            sleep(250);
            robot.goToPosition(-70,16, 0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.goToPosition(-70,16, 90,1);
            sleep(250);
        }
        stop();
    }
}
