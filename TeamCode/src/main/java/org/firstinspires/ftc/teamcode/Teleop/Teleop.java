package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.GamePadConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    private Robot robot;

    double deltaT;
    double timeCurrent;
    double timePre;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        // Servo servo = hardwareMap.servo.get("servo1");

        Robot robot = new Robot(this, timer);
        GamePadConfig gamePadConfig = new GamePadConfig();

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) { // clearer nomenclature for variables
            robot.getGamePadInputs();

            timeCurrent = timer.nanoseconds();


            if (gamepad1.a) {
                if (!gamePadConfig.isbButtonPressedPrev) {
                    robot.control.setIntake(true);
                    if (gamePadConfig.isbButtonPressedPrev) {
                        robot.control.setIntake(false);
                    }
                }
            }

            if (gamepad1.b) {
                if (!gamePadConfig.isbButtonPressedPrev) {
                    robot.control.setIntakeReverse(true);
                }
                if (gamePadConfig.isbButtonPressedPrev) {
                    robot.control.setIntakeReverse(false);
                }
            }
        }
    }
}