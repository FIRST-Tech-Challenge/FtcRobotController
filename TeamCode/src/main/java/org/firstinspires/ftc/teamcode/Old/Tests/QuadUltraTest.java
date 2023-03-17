package org.firstinspires.ftc.teamcode.Old.Tests;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name= "UltraTest")
public class QuadUltraTest extends LinearOpMode {
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    @Override
    public void runOpMode(){
        ElapsedTime op = new ElapsedTime();

        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = hardwareMap.get(AnalogInput.class, "ultrasonicRight");

        ultraRight = hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = hardwareMap.get(LED.class, "ultraLeft");
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        while(opModeIsActive()){
            if(floor(getRuntime())%2==0){
                ultraRight.enable(false);
                ultraLeft.enable(false);
            }
            else{
                ultraRight.enable(true);

                ultraLeft.enable(true);
            }

            telemetry.addData("right", 90.48337 * ultrasonicRight.getVoltage() - 13.12465);
            telemetry.addData("left", 90.48337 * ultrasonicLeft.getVoltage() - 13.12465);

            telemetry.addData("right", ultraRight.isLightOn());
            telemetry.addData("left", ultraLeft.isLightOn());
            telemetry.update();
        }
        stop();
    }



}
