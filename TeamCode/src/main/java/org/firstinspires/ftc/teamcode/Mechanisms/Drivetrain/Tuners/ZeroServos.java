package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import static org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw.clawState.CLOSE;
import static org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw.clawState.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@Autonomous(name = "Zero Servos", group = "Autonomous")
public class ZeroServos extends LinearOpMode {
    Drivetrain drivetrain = null;
    Servo servoWrist;
    Servo servoArmLeft;
    Servo servoArmRight;
    Servo pivotLeft;
    Servo pivotRight;
    Servo clawServo;
    Servo servoExtendLeft;
    Servo servoExtendRight;
    @Override
    public void runOpMode() {
        //servoWrist = hardwareMap.get(Servo.class, "wrist");
        //servoArmLeft = hardwareMap.get(Servo.class, "armRight");
        //servoArmRight = hardwareMap.get(Servo.class, "armLeft");
        pivotLeft = hardwareMap.get(Servo.class, "pivotLeft");
        pivotRight = hardwareMap.get(Servo.class, "pivotRight");
        //clawServo = hardwareMap.get(Servo.class, "clawServo");
        //servoExtendLeft = hardwareMap.get(Servo.class, "leftExtension");
        //servoExtendRight = hardwareMap.get(Servo.class, "rightExtension");
        waitForStart();

        while (opModeIsActive()) {
            //servoArmLeft.setPosition(0);
            //servoArmRight.setPosition(0);
            //servoWrist.setPosition(0.5);
            pivotRight.setPosition(0.5);
            pivotLeft.setPosition(0.5);
            //clawServo.setPosition(0.5);
            //servoExtendLeft.setPosition(0.025);
            //servoExtendRight.setPosition(0);
        }
    }
}