package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="LimitSwitchTest", group="linear")
public class LimitSwitchTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        resetStartTime();
        CRServo reeves = hardwareMap.get(CRServo.class, "reeves");
        reeves.setPower(1);
//        sleep(2000);
//        reeves.setPower(0);
        while(opModeIsActive());
    }

}
