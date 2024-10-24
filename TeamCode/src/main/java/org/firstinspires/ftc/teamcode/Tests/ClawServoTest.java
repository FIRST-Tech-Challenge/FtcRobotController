package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Usefuls.Motor.ServoMotorBetter;
@TeleOp
@Config
// closed: 0.2,opened: 0.8,
public class ClawServoTest extends LinearOpMode {
    public static double angle;
    public void runOpMode(){
        Servo wrist = hardwareMap.get(Servo.class, "claw");
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            wrist.setPosition(angle);
        }

    }
}
