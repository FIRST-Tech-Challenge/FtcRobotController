package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.Robot.op;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
//@Disabled

@Autonomous(name= "touchTest")
public class touchTest extends LinearOpMode {
    private RevTouchSensor limitSwitchRight;
    @Override
    public void runOpMode(){
        limitSwitchRight = hardwareMap.get(RevTouchSensor.class, "limitSwitchRight");
        limitSwitchRight.resetDeviceConfigurationForOpMode();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("isPressed",limitSwitchRight.isPressed());
            telemetry.update();
        }
        stop();
    }
}

