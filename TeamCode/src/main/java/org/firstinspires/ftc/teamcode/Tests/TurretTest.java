package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Turret;

@TeleOp
@Config
//perpendicular counterclockwise: .17, perpendicular clockwise: .84. middle: .5
public class TurretTest extends LinearOpMode {
    public static double t = 0.5;
    public void runOpMode(){
        Turret turret = new Turret(hardwareMap);
        waitForStart();
        turret.setPosition(0.5);
        while(opModeIsActive()&&!isStopRequested()){
            turret.setPosition(t);
        }

    }
}
