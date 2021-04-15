package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "CorgiFinalAutoHighGoal", preselectTeleOp = "OneGPTeleop")

public class CorgiFinalAutoHighGoal extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        ElapsedTime runtime = new ElapsedTime();
//        int rings = robot.getRingsAndWaitForStart();
        int rings = 1;
        robot.navigate();
        robot.stopRingDetection();
        telemetry.addData("ReadyToStart",0);
        telemetry.update();
//        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        waitForStart();
        if(rings!=1&&rings!=4) {
        // 0 RING AUTO
            robot.goToPosition(-23,-10,-13.5,0.9);
//            robot.shootHighGoal(3);
            robot.goToPosition(-62,-15,0,1);
//            robot.openWobbleGoalClaw();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-5,39,45,0.8);
            robot.goToPosition(-2,20,90,0.8);
//            robot.closeWobbleGoalClaw();
            sleep(600);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-62,-12,0,0.8);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
//            sleep(250);
            robot.goToPosition(-66, -12,0,0.5);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//            robot.turnInPlace(180,0.7);
        }
        else if(rings==1) {
        // 1 RING AUTO
            robot.goToPosition(-50,-16,0,0.9);
            robot.goToPosition(-55,4,0,0.6);
            robot.shootHighGoal(3);
            sleep(500);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-45,5,0.5,0.6);
            sleep(250);
            robot.shootHighGoal(1);
            sleep(250);
            robot.stopIntake();
            robot.stopTransfer();
            sleep(250);
            robot.goToPosition(-86,4,0,1);
//            robot.openWobbleGoalClaw();
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-5,39,45,0.8);
            robot.goToPosition(-2,20,90,0.8);
//            robot.closeWobbleGoalClaw();
            sleep(600);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-86,4,0,0.8);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
//            sleep(250);
            robot.goToPosition(-74,17.5,0,0.6);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//            robot.turnInPlace(180,0.7);
        }
        else if(rings==4) {
        // 4 RING AUTO
            robot.goToPosition(-23,-10,-13.5,0.9);
//            robot.shootHighGoal(3);
            robot.goToPosition(-110, -15, 0,1);
//            robot.openWobbleGoalClaw();
            robot.goToPosition(-60,-2,1,0.6);
//            robot.startIntake();
//            robot.startTransfer();
            robot.goToPosition(-45,-1,0.5,0.5);
            robot.goToPosition(-43,0,-2,0.4);
            sleep(500);
//            robot.shootHighGoal(1);
//            sleep(250);
            robot.goToPosition(-33,0,0,1);
            robot.goToPosition(-33,0,-2,1);
//            robot.shootHighGoalTest(1670, 72,3);
//            robot.stopIntake();
//            robot.stopTransfer();
            sleep(250);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-5,39,45,0.8);
            robot.goToPosition(-2,20,90,0.8);
//            robot.closeWobbleGoalClaw();
            sleep(600);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-115,-12,0,0.8);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
//            robot.openWobbleGoalClaw();
//            sleep(250);
            robot.goToPosition(-70,16, 0,1);
//            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);

//            sleep(250);
        }
        stop();
    }
}
