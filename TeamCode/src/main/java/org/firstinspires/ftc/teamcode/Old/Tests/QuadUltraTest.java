package org.firstinspires.ftc.teamcode.Old.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;


@Autonomous(name= "RAAAAAAAAAAAAAAAAAAAAAAAAHHH")
public class QuadUltraTest extends LinearOpMode {
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    @Override
    public void runOpMode(){
//        logger.createFile("/RobotLogs/GeneralRobot", "Runtime    Component               " +
//                "Function                        Action");
        BasicRobot robot = new BasicRobot(this,false);
        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = hardwareMap.get(AnalogInput.class, "ultrasonicRight");

//        ultraRight = hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = hardwareMap.get(LED.class, "ultraLeft");
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        while(opModeIsActive()){
            if(floor(getRuntime())%2==0){
//                ultraRight.enable(false);
                ultraLeft.enable(false);
            }
            else{
//                ultraRight.enable(true);

                ultraLeft.enable(true);
            }

            op.telemetry.addData("rightS", 90.48337 * ultrasonicRight.getVoltage() - 12.62465);
            op.telemetry.addData("leftS", 90.48337 * ultrasonicLeft.getVoltage() - 12.62465);

//            op.telemetry.addData("right", ultraRight.isLightOn());
            op.telemetry.addData("left", ultraLeft.isLightOn());
            telemetry.update();
        }
        stop();
    }



}
