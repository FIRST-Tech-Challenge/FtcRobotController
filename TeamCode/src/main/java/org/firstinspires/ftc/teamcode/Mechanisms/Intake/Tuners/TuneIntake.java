package org.firstinspires.ftc.teamcode.Mechanisms.Intake.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@Autonomous(name = "Test Intake", group = "Autonomous")
public class TuneIntake extends LinearOpMode {
    Drivetrain drivetrain = null;
    public static double speed = 1;
    Battery battery;
    @Override
    public void runOpMode(){
        battery = new Battery(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, battery);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a) {
                intake.setPower(speed);
                intakeServo.setPower(-1);
            }
            if (gamepad1.b){
                intake.setPower(-speed*.4);
                intakeServo.setPower(1);
            }
        }
    }
}
