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
        int rings=0;
        if(rings!=1&&rings!=4) {
            robot.goToPosition(-10,0,0,2);
            robot.goToPosition( -26,-16,-183,2);
//            robot.openWobbleGoalClaw();
            robot.goToPosition(-24,33,-2.5,1.5);
            telemetry.addData("shoot pshots", 0);
            telemetry.update();
            sleep(300);
//            robot.shootThreePowerShot();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.goToPosition(-15, 23,0,2);
            robot.goToPosition(-11,23, 0,2);
//            robot.closeWobbleGoalClaw();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-23,-14,180,2);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-15, -15,178,1);
            robot.goToPosition(-15,-6,178,2);
            robot.goToPosition(-96,-3,90,2);
//            robot.startIntake();
//            robot.startTransfer();
            robot.goToPosition(-96, 17,90,2);
            sleep(100);
            robot.goToPosition(-77,17,90,1.5);
            sleep(1000);
            robot.goToPosition(-77,7,-90,2);
//            robot.stopIntake();
//            robot.stopTransfer();
            sleep(3000);
            robot.goToPosition(-45, -2,3,2);
//            robot.shootHighGoal(3);
            robot.goToPosition(-56,10,180,2);
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
            robot.goToPosition(-75, -1, 90,2);
            sleep(300);
            telemetry.update();
            sleep(400);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
            robot.goToPosition(-75, -4,90,1);
            robot.goToPosition(-90, -4,90,1);
//            robot.startIntake();
//            robot.startTransfer();
            robot.goToPosition(-90, 17,90,2);
//            robot.stopIntake();
//            robot.stopTransfer();
            robot.goToPosition(-85,17,90,2);
            robot.goToPosition(-45, 19,0,2);
            robot.goToPosition(-40,19, 0,1);
//            robot.closeWobbleGoalClaw();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            sleep(300);
            robot.goToPosition(-55, 15,4,1);
//            robot.shootHighGoal(3);
            sleep(300);
            robot.goToPosition(-64, -2, 180,0.9);
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
