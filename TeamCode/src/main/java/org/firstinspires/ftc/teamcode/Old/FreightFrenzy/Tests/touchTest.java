package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Tests;


import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled

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

