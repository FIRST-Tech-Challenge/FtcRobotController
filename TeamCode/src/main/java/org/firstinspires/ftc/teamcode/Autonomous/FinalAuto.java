package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "FinalAuto")
public class FinalAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        if(rings!=1&&rings!=4) {
            robot.goToPosition( -64,-6,0, 1);//-25,60
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-60.5,30.8,-2,0.8);//37,4
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-53.5+44,32+3,0,0.8);
            robot.turnInPlace(0,0.6);
            robot.goToPosition(-53.5+44,32+3-6.9,0,0.8);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-53.5+43.5-56.5-7,32+3-6.5-28,0,0.8);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-53.5+43.5-59-2,32+6.5-4-23+10,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.turnInPlace(180,0.7);
        }
        else if(rings==1) {
            robot.goToPosition(-98,12,0,1);
            robot.openWobbleGoalClaw();
            sleep(200);
            robot.goToPosition(-60.5,25.25,-2,0.7);//yPosition - 23
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-53.5+44,30,0,1);//yPosition - 26
            robot.turnInPlace(0,1.0);
            robot.goToPosition(-53.5+44,22,0,0.8);
            sleep(200);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-88,12,0,1);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-72,17,0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);
            robot.turnInPlace(180,0.7);
        }
        else if(rings==4) {
            robot.goToPosition(-30,-9.5,-16,1);
            robot.shootGoalTeleop(120);
            robot.shootHighGoal(3);
            robot.goToPosition(-112, 0, 0,1);
            robot.openWobbleGoalClaw();
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-63,6,0,0.8);
            robot.goToPosition(-62.6,4,0,0.3);
            sleep(1000);
            robot.stopIntake();
            robot.stopTransfer();
            robot.goToPosition(-49,4,0,0.8);
            robot.goToPosition(-49.25,4,-3,1);
            robot.shootHighGoal(2);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-45,2,-5,1);
            robot.shootHighGoal(2);
            robot.goToPosition(-70,1, 0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(420);
        }
        stop();
    }
}
