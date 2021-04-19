package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "CorgiFinalAutoHighGoal", preselectTeleOp = "OneGPTeleop")

public class CorgiFinalAutoHighGoal extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        int rings = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
//        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        telemetry.addData("ReadyToStart",0);
        telemetry.update();
        if(rings!=1&&rings!=4) {
        // 0 RING AUTO
            robot.goToPosition(-54,4,0,0.75);
            robot.shootHighGoal(3);
            robot.goToPosition(-62,-15,0,1);
            robot.openWobbleGoalClaw();
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-5,39,45,0.8);
            robot.goToPosition(-11,25,90,0.8);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-62,-12,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-66, -12,0,0.5);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//            robot.turnInPlace(180,0.7);
        }
        else if(rings==1) {
        // 1 RING AUTO
            robot.goToPosition(-55,-16,0,0.95);
            robot.goToPosition(-56,4,0,0.75);
            robot.shootHighGoal(1);
            sleep(500);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-47,4,0.5,0.6);
            sleep(250);
            robot.stopIntake();
            robot.shootHighGoal(3);
            robot.goToPosition(-70,5,0,1);
            robot.stopTransfer();
            robot.goToPosition(-85,6,0,1);
            sleep(250);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.DROP);
            robot.openWobbleGoalClaw();
            robot.goToPosition(-80,6,0,0.7);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-5,39,45,0.8);
            robot.goToPosition(-11,25,90,0.8);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-86,4,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-74,17.5,0,0.6);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
//            robot.turnInPlace(180,0.7);
        }
        else if(rings==4) {
        // 4 RING AUTO
            robot.goToPosition(-55,-16,0,0.95);
            robot.goToPosition(-55,4,0,0.75);
            robot.shootHighGoal(3);
            sleep(500);
            robot.startIntake();
            robot.startTransfer();
            robot.goToPosition(-49,4,0,0.5);
            sleep(250);
            robot.shootHighGoal(1);
            sleep(100);
            robot.goToPosition(-29,4,0,0.41);
            sleep(250);
            robot.goToPosition(-49,4,0,0.41);
            robot.shootHighGoal(3);
            robot.stopIntake();
            robot.goToPosition(-73,-16,0,1);
            robot.stopTransfer();
            robot.goToPosition(-102,-16,0,1);
            sleep(250);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.DROP);
            robot.openWobbleGoalClaw();
            sleep(100);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            //line below is to just go to parking line
            robot.goToPosition(-74,17.5,0,0.75);
            /*
            second wg code
            robot.turnInPlace(0, 1.0);
            robot.goToPosition(-5,39,45,0.8);
            robot.goToPosition(-11,25,90,0.8);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-115,-12,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.AutoGRAB);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-74,17.5,0,0.6);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);
             */
        }
        stop();
    }
}
