package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;

@TeleOp(name = "Teleop")
@Disabled

public class Teleop extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;

    enum Prospective {
        ROBOT,
        DRIVER,
    }

    //    enum MainClawState {
//        CLOSE,
//        OPEN,
//        WIDEOPEN,
//    }
    private double robotAngle;
    private boolean visionEnabled = false;
    private boolean wobbleClawControlDigital = true;
    private boolean wobbleClawDeployed = false;
    private boolean wobbleClawOpen = false;

    private boolean isIntakeOn = false;
    private boolean isLaunchOn = false;





    private void initOpMode() {
        //Initialize DC motor objects
        timer = new ElapsedTime();
        try {
            robot = new Robot(this, timer, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        waitForStart();

//        robot.initServosTeleop();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        while(opModeIsActive()) {

            // Get gamepad inputs
            robot.getGamePadInputs();

            // Get the current time
            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            // Drive the motors
            double[] motorPowers;
            robotAngle = robot.imu.getAngularOrientation().firstAngle;

            motorPowers = robot.drive.calcMotorPowers(robot.leftStickX, robot.leftStickY, robot.rightStickX);
            robot.drive.setDrivePowers(motorPowers);

            // reset drive motor encoders
            if (robot.yButton && !robot.isyButtonPressedPrev) {
                robot.drive.resetDriveMotorEncoders();
            }

//            //Toggle intake
//            if (robot.aButton && !robot.isaButtonPressedPrev){
//                if(isIntakeOn){
//                    robot.control.setIntake(false);
//                    isIntakeOn = false;
//                }
//                else{
//                    robot.control.setIntake(true);
//                    isIntakeOn = true;
//                }
//            }
//
//            //Toggle launcher
//            if (robot.bButton && !robot.isbButtonPressedPrev){
//                if(isLaunchOn) {
//                    robot.control.setLaunch(false);
//                    isLaunchOn = false;
//                }
//                else{
//                    robot.control.setLaunch(true);
//                    isLaunchOn = true;
//                }
//
//            }
//
////            int currentPositions[] = robot.drive.getCurrentPositions();
////            telemetry.addData("position", "fl %d, fr %d, rl %d, rr %d",
////                    currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]);
            telemetry.update();
        }
    }

}