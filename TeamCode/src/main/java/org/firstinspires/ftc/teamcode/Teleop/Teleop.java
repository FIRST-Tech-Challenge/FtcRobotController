package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.GamePadConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;


import java.io.IOException;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    private Robot robot;

    double deltaT;
    double timeCurrent;
    double timePre;

    ElapsedTime timer;

    private double robotAngle;
    private boolean isIntakeOn = false;

    private void initOpMode() {
        //Initialize DC motor objects
        timer = new ElapsedTime();
        robot = new Robot(this, timer);

        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        ElapsedTime timer = new ElapsedTime();

        GamePadConfig gamePadConfig = new GamePadConfig();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        while (opModeIsActive()) { // clearer nomenclature for variables
            robot.getGamePadInputs();

            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            double[] motorPowers;
            robotAngle = robot.imu.getAngularOrientation().firstAngle;
            motorPowers = robot.drive.calcMotorPowers(robot.gamePadConfig.leftStickX*0.5, robot.gamePadConfig.leftStickY*0.5, robot.gamePadConfig.rightStickX*0.5);
            robot.drive.setDrivePowers(motorPowers);

            robot.drive.setDrivePowers(motorPowers);

            //Toggle intake regular
            if (robot.gamePadConfig.aButton && !robot.gamePadConfig.isaButtonPressedPrev){
                if(isIntakeOn){
                    robot.control.setIntakeDirection(true);
                    isIntakeOn = false;
                }
                else {
                    robot.control.setIntakeDirection(true);
                    isIntakeOn = true;
                }
            }

            //Toggle intake reverse
            if (robot.gamePadConfig.bButton && !robot.gamePadConfig.isbButtonPressedPrev){
                if(isIntakeOn){
                    robot.control.setIntakeDirection(false);
                    isIntakeOn = false;
                }
                else {
                    robot.control.setIntakeDirection(false);
                    isIntakeOn = true;
                }
            }
        }
    }
}