package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;

@Config
@Autonomous(name = "Test Intake", group = "Autonomous")
public class TuneIntake extends LinearOpMode {
    Drivetrain drivetrain = null;
    public static double speed = 1;
    @Override
    public void runOpMode(){
        drivetrain = new Drivetrain(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "lfm");
        Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a) {
                intake.setPower(speed);
                intakeServo.setPosition(0.5);
            }
            if (gamepad1.b){
                intake.setPower(-speed*.4);
                intakeServo.setPosition(-0.5);
            }
        }
    }
}
