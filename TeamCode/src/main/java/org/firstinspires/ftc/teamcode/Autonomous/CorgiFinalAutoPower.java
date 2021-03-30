package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.OdometryChassis;

@Autonomous(name= "CorgiFinalAutoPower", preselectTeleOp = "OneGPTeleop")

public class CorgiFinalAutoPower extends LinearOpMode {
    @Override
    public void runOpMode(){
        OdometryChassis robot = new OdometryChassis(this, false, true);
//        int rings = robot.getRingsAndWaitForStart();
//        robot.navigate();
//        robot.stopRingDetection();
//        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        telemetry.addData("ReadyToStart",0);
        telemetry.update();
        waitForStart();
        int rings=1;
        if(rings!=1&&rings!=4) {
            robot.goToPosition( -66,-6,0, 1);//-25,60
//            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-58.5,33.8,0,0.8);//37,4
//            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-53.5+43,39,0,0.8);
            robot.turnInPlace(0,0.6);
            robot.goToPosition(-53.5+43.5,32+10.5-5.8-3,0,0.8);
//            robot.closeWobbleGoalClaw();
            sleep(600);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-53.5+43.5-56.5-7,32+3-6.5-24,0,0.8);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-53.5+43.5-56.5-7,32+3-6.5-24+6,0,0.7);
            robot.goToPosition(-100, 32+3-6.5-24+6+12,-90,1);
//            robot.startIntake();
//            robot.startTransfer();
            robot.goToPosition(-100, 32+3-6.5-24+6+12+40,-90,1);
            robot.goToPosition(-53.5+43.5-59-5,32+6.5-4-23+6,0,0.6);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.turnInPlace(180, 1);
        }
        else if(rings==1) {
            robot.goToPosition(-20,-10,0,2);
            robot.goToPosition(-30,-10,0,2);
            robot.goToPosition(-30,4,0,2);
//            robot.shootHighGoal(1);
            telemetry.addData("shoot high goal", 0);
            telemetry.update();
            sleep(100);
//            robot.startIntake();
//            robot.startTransfer();
            robot.goToPosition(-24, 4, 0, 2);
//            robot.stopIntake();
//            robot.stopTransfer();
            robot.goToPosition(-26,28,-2.5,1);
            sleep(100);
            telemetry.addData("shoot pshots", 0);
            telemetry.update();
            sleep(300);
//            robot.shootThreePowerShot();
            telemetry.addData("wobble goal_1", 0);
            robot.goToPosition(-75, -4, 90,2);
            sleep(300);
            telemetry.update();
            sleep(400);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
            robot.goToPosition(-75, -7.5,90,1);
            robot.goToPosition(-90, -7.5,90,1);
//            robot.startIntake();
//            robot.startTransfer();
            robot.goToPosition(-90, 17,90,2);
//            robot.stopIntake();
//            robot.stopTransfer();
            robot.goToPosition(-85,17,90,2);
            robot.goToPosition(-45, 16,0,2);
            robot.goToPosition(-40,16, 0,1);
//            robot.closeWobbleGoalClaw();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            sleep(300);
            robot.goToPosition(-55, 10,4,1);
//            robot.shootHighGoal(3);
            sleep(300);
            robot.goToPosition(-58, -1, 180,0.9);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-56,-1,180,1);
//            robot.goToPosition(-56,15,0,1);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);

            /***Alternate below***/

//            robot.turnInPlace(180,0.7);
//
//            robot.goToPosition(-100,40,-90,0.9);
////            robot.startIntake();
////            robot.startTransfer();
//            robot.goToPosition(-100, 10,-90,1);
//            robot.goToPosition(-90,10,0,1);
////            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
////            robot.openWobbleGoalClaw();
////            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
//            robot.goToPosition(-53.5+43,27,-90,0.8);
////            robot.closeWobbleGoalClaw();
////            sleep(600);
////            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
//            robot.goToPosition(-92,19.5,0,1);
////            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
////            robot.openWobbleGoalClaw();
//            sleep(250);
//            robot.goToPosition(-72,23,0,1);
////            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//            sleep(200);
//            robot.turnInPlace(180,0.7);
        }
        else if(rings==4) {
            robot.goToPosition(-23,-10,-13.5,0.9);
//            robot.shootHighGoalTest(1670, 100,3);
            robot.goToPosition(-115, -7, 0,1);
//            robot.openWobbleGoalClaw();
//            robot.goToPosition(-60,-2,1,0.6);
//            robot.startIntake();
//            robot.startTransfer();
            robot.goToPosition(-45,-1,0.5,0.5);
            robot.goToPosition(-43,0,-2,0.4);
            sleep(500);
//            robot.shootHighGoal(1);
//            robot.startIntake();
//            robot.startTransfer();
            sleep(250);
            robot.goToPosition(-33,0,0,1);
            robot.goToPosition(-33,0,-2,1);
//            robot.stopIntake();
//            robot.stopTransfer();
            sleep(250);
            robot.goToPosition(-54.5,31,-2.5,0.7);//yPosition - 23
//            robot.shootThreePowerShot();
            robot.goToPosition(-18, -9,-180,1);
            robot.goToPosition(-9,-10, -180,1);
//            robot.closeWobbleGoalClaw();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(100,-2,0,1);
//            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-72,23,0,1);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);
            robot.turnInPlace(180,0.7);
        }
        stop();
    }
}
