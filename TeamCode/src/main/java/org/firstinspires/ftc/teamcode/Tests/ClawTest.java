package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Claw;

@TeleOp
@Config
// closed: 0.2,opened: 0.8,
public class ClawTest extends LinearOpMode {
    public static Boolean open=false;
    public static double c;
    public void runOpMode(){
        //Servo claw = hardwareMap.get(Servo.class, "claw");
        Claw claw = new Claw(hardwareMap);
        waitForStart();
        claw.setPosition(0.2);
        while(opModeIsActive()&&!isStopRequested()){
            claw.setPosition(c);
            if (isStopRequested()){
                claw.close();
                break;
            }
        }

    }
}
