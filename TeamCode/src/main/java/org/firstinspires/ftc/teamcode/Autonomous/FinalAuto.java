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
            robot.goToPosition( -66,-0,0, 1);//-25,60
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-60.5,37.3,-2,0.8);//37,4
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-53.5+44,32+12,0,0.8);
            robot.turnInPlace(0,0.6);
            robot.goToPosition(-53.5+44,32+10.5-5.8,0,0.8);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-53.5+43.5-56.5-7,32+3-6.5-28,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-53.5+43.5-59-2,32+6.5-4-23+10,0,0.6);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.turnInPlace(180,0.7);
        }
        else if(rings==1) {
            robot.goToPosition(-96,12,0,1);
            robot.openWobbleGoalClaw();
            sleep(200);
            robot.goToPosition(-59.5,25.25,-2,0.7);//yPosition - 23
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-53.5+44,36,0,1);//yPosition - 26
            robot.turnInPlace(0,1.0);
            robot.goToPosition(-53.5+44,29,0,0.8);
            sleep(200);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-88,23,0,1);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-72,19,0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);
            robot.turnInPlace(180,0.7);
        }
         else if(rings==4) {
            robot.goToPosition(-23,-21,-13,1);
            robot.shootHighGoalTest(1645, 100,3);
            robot.goToPosition(-115, -10, 0,1);
            robot.openWobbleGoalClaw();
            robot.goToPosition(-55,-7,0,1);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-50,-7,0,0.65);
            robot.goToPosition(-47,-7,0,0.4);
            sleep(500);
            robot.goToPosition(-51,-7,-2,0.9);
            robot.shootHighGoal(1);
//            robot.startIntake();
//            robot.startTransfer();
            sleep(250);
            robot.goToPosition(-52,-7,0,1);
            robot.goToPosition(-38,-7,0,1);
            robot.goToPosition(-38,-7,-1,1);
            robot.shootHighGoal(3);
            robot.stopIntake();
            robot.stopTransfer();
            sleep(250);
            robot.goToPosition(-70,10, 0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.goToPosition(-70,10, -180,1);
            sleep(250);
        }
        stop();
    }
}
