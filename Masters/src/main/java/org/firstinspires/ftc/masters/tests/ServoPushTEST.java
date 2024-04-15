package org.firstinspires.ftc.masters.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.CSCons.ClawPosition;
import org.firstinspires.ftc.masters.CSCons.DriveMode;
import org.firstinspires.ftc.masters.CSCons.HookPosition;
import org.firstinspires.ftc.masters.CSCons.IntakeState;
import org.firstinspires.ftc.masters.CSCons.OuttakePosition;
import org.firstinspires.ftc.masters.CSCons.OuttakeState;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "SERVO PUsh TEST", group = "test")
public class ServoPushTEST extends LinearOpMode {


    Servo push;


    @Override
    public void runOpMode() {



        push = hardwareMap.servo.get("push");


        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                push.setPosition(0.1);
            }
            if (gamepad1.b){
                push.setPosition(0.58);
            }
        }





    }


}
