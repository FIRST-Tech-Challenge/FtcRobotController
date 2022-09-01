package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name= "QuadUltraTest")
public class QuadUltraTest extends LinearOpMode {
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    @Override
    public void runOpMode(){
        ElapsedTime op = new ElapsedTime();
        ultrasonicFront = hardwareMap.get(AnalogInput.class, "ultrasonicFront");
        ultrasonicBack = hardwareMap.get(AnalogInput.class, "ultrasonicBack");
        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = hardwareMap.get(AnalogInput.class, "ultrasonicRight");
        ultraFront = hardwareMap.get(LED.class, "ultraFront");
        ultraBack = hardwareMap.get(LED.class, "ultraBack");
        ultraRight = hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = hardwareMap.get(LED.class, "ultraLeft");
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        while(opModeIsActive()){
            if(floor(getRuntime())%2==0){
                ultraBack.enable(false);
                ultraRight.enable(false);
                ultraFront.enable(false);
                ultraLeft.enable(false);
            }
            else{
                ultraBack.enable(true);
                ultraRight.enable(true);
                ultraFront.enable(true);
                ultraLeft.enable(true);
            }
            telemetry.addData("front", 90.48337 * ultrasonicFront.getVoltage() - 13.12465);
            telemetry.addData("back", 90.48337 * ultrasonicBack.getVoltage() - 13.12465);
            telemetry.addData("right", 90.48337 * ultrasonicRight.getVoltage() - 13.12465);
            telemetry.addData("left", 90.48337 * ultrasonicLeft.getVoltage() - 13.12465);
            telemetry.addData("front", ultraFront.isLightOn());
            telemetry.addData("back", ultraBack.isLightOn());
            telemetry.addData("right", ultraRight.isLightOn());
            telemetry.addData("left", ultraLeft.isLightOn());
            telemetry.update();
        }
        stop();
    }



}
