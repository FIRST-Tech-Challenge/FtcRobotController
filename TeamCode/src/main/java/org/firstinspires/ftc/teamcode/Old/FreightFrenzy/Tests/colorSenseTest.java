package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.ColorDistanceRevV3;
@Disabled
@Autonomous(name= "colorPrint")
public class colorSenseTest extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        //Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);
        ElapsedTime op = new ElapsedTime();
        ColorDistanceRevV3 color = new ColorDistanceRevV3();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //ElapsedTime runtime = new ElapsedTime();
        //int rings = robot.getRingsAndWaitForStart();
        //robot.stopRingDetection();
        waitForStart();
        while(opModeIsActive()){
            float[] hsv =color.hsvVal();
            telemetry.addData("h",hsv[0]);
            telemetry.addData("s",hsv[1]);
            telemetry.addData("v",hsv[2]);
            telemetry.addData("r",color.red());
            telemetry.addData("g",color.green());
            telemetry.addData("b",color.blue());
            telemetry.update();
        }
        stop();
    }



}
