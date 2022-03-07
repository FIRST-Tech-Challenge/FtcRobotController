package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.Components.VSLAMChassis.barrier;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "ScrimAuto", preselectTeleOp = "OneGPTeleop")
public class ScrimAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);
        //int position = robot.BlueElemTest(this,0,0);
        double[] turretTarget = {0,0,0};//{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        waitForStart();
//        robot.TurretSlidesToPosition(-11.5,10.0,0,0.5);
        robot.goToPosition(0,-28,-10,-15,0.5);
//        robot.FlipBasketArmToPosition(.5);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.01);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.5);
//        sleep(500);
//        robot.TurretSlidesToPosition(0,0,0,0.5);
        robot.goToPosition(1,-9.4,-27.2,-30,0.4);
//        robot.FlipBasketArmToPosition(0.00);
//        robot.FlipBasketToPosition(1.0);
        robot.spinCarouselAutonomousBlue();
        /** 3, -49; 19.5, -62.3; 36, -31; **/
        robot.partOfPolySplineToPositionHead(0,-25.5,-5.6,-25.5,-5.6,2,-56,30,-66,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0,-25.6,-5.6, 2,-56,30,-66,35,-26,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0, 2,-56,30,-66,35,-29,35,-29,true,true,0.5);
        robot.turnInPlace(90,0.7);
        robot.goToPositionTeleop(1, 68, -28, 1.0);
        if(!barrier){
            stop();
        }
        robot.goToPosition(1,-29,84, 0 ,0.5);
//        robot.partOfPolySplineToPositionHead(1,68,-32, 81,-27,99,-21,99,-12,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(1, 81,-27,99,-21,99,-16, 99,0,true,true,0.5);
        if(!robot.autoIntake(0.2,10)){
            stop();
        }
        robot.goToPosition(0,-29,84,90,0.5);
        robot.goToPositionTeleop(0,34,-28,1.0);
        if(!barrier){
            stop();
        }
        robot.turnInPlace(75,0.5);
        sleep(2000);
        robot.turnInPlace(90,0.5);
        robot.goToPositionTeleop(1, 68, -28, 1.0);
        if(!barrier){
            stop();
        }
        stop();
    }
}


