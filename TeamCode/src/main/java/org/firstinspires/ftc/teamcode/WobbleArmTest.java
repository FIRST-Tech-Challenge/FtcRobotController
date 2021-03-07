package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot.Hitter;

@Config
class WobbleArmServoVals {
    public static double position = 0;
}

@TeleOp (name = "WobbleArmServoTest")
public class WobbleArmTest extends LinearOpMode{

    private Servo wobbleArmServo;
    double servoPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");
        wobbleArmServo.setPosition(servoPosition);

        waitForStart();

        servoPosition = 0.5;
        wobbleArmServo.setPosition(servoPosition);
        sleep(2000);

        servoPosition = 1.0;
        wobbleArmServo.setPosition(servoPosition);
        sleep(2000);


    }
}

