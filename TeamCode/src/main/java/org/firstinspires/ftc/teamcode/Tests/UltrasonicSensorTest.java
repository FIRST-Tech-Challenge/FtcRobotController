package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RangeSensor;


@Autonomous(name= "UltrasonicTest")
public class UltrasonicSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
//        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, true,90);
        RangeSensor ultrasonic = new RangeSensor(this);
//        LED ultraFront =hardwareMap.get(LED.class, "ultraFront");
        ElapsedTime op = new ElapsedTime();
        double lastState=0.0;
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("voltage", ultrasonic.getVoltage(true));
            telemetry.addData("distance", ultrasonic.getDistance(true));
            telemetry.addData("voltage", ultrasonic.getVoltage(false));
            telemetry.addData("distance", ultrasonic.getDistance(false));
            telemetry.addData("state",ultrasonic.getState(true));
            telemetry.addData("lastState",lastState);
            telemetry.addData("runtime",getRuntime());
            telemetry.addData("x", ultrasonic.getLocation()[0]);
            telemetry.addData("y", ultrasonic.getLocation()[1]);
            telemetry.update();
            if(getRuntime()-lastState>.01){
                lastState=getRuntime();
//                ultrasonic.setState(true,false);
                if(ultrasonic.getState(true)) {
                    ultrasonic.setState(true, false);
                }
                else{
                    ultrasonic.setState(true,true);
                }
            }

//            robot.track();
        }
        stop();
    }
}
