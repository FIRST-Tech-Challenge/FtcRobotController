package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="LimitSwitchTest", group="linear")
public class LimitSwitchTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        resetStartTime();
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "limit");
        while(opModeIsActive()) {
            if(touchSensor.isPressed()) {
                telemetry.addData("Pressed? ", "true");
                telemetry.update();
            }else{
                telemetry.addData("Pressed? ", "false");
                telemetry.update();
            }
        }
    }

}
