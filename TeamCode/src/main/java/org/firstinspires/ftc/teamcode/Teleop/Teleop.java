package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Subsystems.IntakeToElevatorThread;
//import org.firstinspires.ftc.teamcode.Subsystems.LauncherFeederThread;
//import org.firstinspires.ftc.teamcode.Subsystems.LauncherThread;
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.Subsystems.WobbleGoalArmTeleopThread;

import java.io.IOException;

/**
 * Created by Andrew Chiang on 4/18/2021
 */

@TeleOp(name = "TeleopMark4")
public class TeleopMark extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    double deltaT;
    double timeCurrent;
    double timePre;

    double leftStickY;
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

    private boolean driveHighPower = true;

    private boolean isIntakeOn = false;
    private boolean isLaunchOn = false;
    private boolean isIntakeToElevator = false;
    private boolean isLauncherFeeder = false;
    private boolean isWobbleClawOpen = false;
    private double  wobbleGoalArmIncrement = 0.0;

    private double launchRPMHighGoal = 1600.0;
    private double launchRPMPowerShot = 1500.0;
    private double launchCurrentRPMTarget = launchRPMHighGoal;
    private double launchCurrentRPM;
    private double newLaunchRPMTarget;
    private double incrementRPM = 20.0;

    private boolean targetVisible = false;
    private static final int MAX_ITERATION = 40;
    private static final double towerOffsetTargetX = -360.0;
    private static final double towerOffsetTargetY = -1743.0;
    private static final double towerOffsetTargetAngle = -0.6;

    private static final float mmPerInch        = 25.4f;
    private double launchPosX = 11.0;

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

        robot.control.restLauncherFeeder();
        robot.control.openIntakeToElevator();
        robot.control.moveElevatorToBottom();
        robot.wobbleGoalArm.setPosition(0.0);
//        robot.control.moveWobbleGoalArmDown();
        robot.control.openWideWobbleGoalClaw();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        Thread intakeToElevatorThread = new IntakeToElevatorThread(this, robot);
        Thread launcherFeederThread = new LauncherFeederThread(this, robot);
        Thread wobbleGoalArmThread = new WobbleGoalArmTeleopThread(this, robot);
        Thread launcherThread = new LauncherThread(this, robot);
        robot.control.setLauncherTargetRPM(0.0);

        waitForStart();
        wobbleGoalArmThread.start();
        launcherFeederThread.start();
        intakeToElevatorThread.start();
        launcherThread.start();

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
            if (driveHighPower) {
                motorPowers = robot.drive.calcMotorPowers(robot.leftStickX, robot.leftStickY, robot.rightStickX);
            }
            else {
                motorPowers = robot.drive.calcMotorPowers(robot.leftStickX*0.5, robot.leftStickY*0.5, robot.rightStickX*0.5);
            }
            robot.drive.setDrivePowers(motorPowers);

            //Toggle intake regular
            if (robot.aButton && !robot.isaButtonPressedPrev){
                if(isIntakeOn){
                    robot.control.setIntake(false);
                    isIntakeOn = false;
                }
                else{
                    robot.control.moveElevatorToBottom();
                    robot.control.setIntake(true);
                    isIntakeOn = true;
                }
            }

            //Toggle intake REVERSE
            if (robot.bButton && !robot.isbButtonPressedPrev){
                if(isIntakeOn){
                    robot.control.setIntakeReverse(false);
                    isIntakeOn = false;
                }
                else{
                    robot.control.setIntakeReverse(true);
                    isIntakeOn = true;
                }
            }

            //Toggle launcher
            if (robot.xButton && !robot.isxButtonPressedPrev){
                if(isLaunchOn) {
                    robot.control.setLauncherTargetRPM(0.0);
                    isLaunchOn = false;
                }
                else{
                    robot.control.setLauncherTargetRPM(launchCurrentRPMTarget);
                    isLaunchOn = true;
                }
            }

            //Toggle drive power
            if (robot.yButton && !robot.isyButtonPressedPrev){
                if(driveHighPower) {
                    driveHighPower = false;
                }
                else{
                    driveHighPower = true;
                }
            }

            //Move elevator
            if(robot.dPadUp && !robot.isdPadUpPressedPrev && (robot.control.getElevatorStage() != 0)){
                robot.control.moveElevator(1);
            }

            if(robot.dPadDown && !robot.isdPadDownPressedPrev){
                robot.control.moveElevator(-1);
            }

            //toggle wobble goal claw
            if(robot.bumperLeft && !robot.islBumperPressedPrev){
                if(isWobbleClawOpen){
                    robot.control.closeWobbleGoalClaw();
                    isWobbleClawOpen = false;
                }
                else{
                    robot.control.openWobbleGoalClaw();
                    isWobbleClawOpen = true;
                }

            }

            if((robot.triggerLeft > 0.3)) {
                autoLaunchHighGoal();
            }

            if((robot.triggerLeft2 > 0.3)) {
                autoLaunchPowerShot();
            }


            //adjust launch RPM
            if(robot.dPadRight && !robot.isdPadRightPressedPrev && isLaunchOn){
                newLaunchRPMTarget = launchCurrentRPMTarget + incrementRPM;
                if(newLaunchRPMTarget > robot.control.getLauncherRPMLimit()){
                    newLaunchRPMTarget = robot.control.getLauncherRPMLimit();
                }
                robot.control.setLauncherTargetRPM(newLaunchRPMTarget);
                launchCurrentRPMTarget = newLaunchRPMTarget;

            }
            if(robot.dPadLeft && !robot.isdPadLeftPressedPrev && isLaunchOn){
                newLaunchRPMTarget = launchCurrentRPMTarget - incrementRPM;
                if(newLaunchRPMTarget < 0.0){
                    newLaunchRPMTarget = 0.0;
                }
                robot.control.setLauncherTargetRPM(newLaunchRPMTarget);
                launchCurrentRPMTarget = newLaunchRPMTarget;
            }

//            int currentPositions[] = robot.drive.getCurrentPositions();
//            telemetry.addData("position", "fl %d, fr %d, rl %d, rr %d",
//                    currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]);
            launchCurrentRPM = robot.control.getLauncherCurrentRPM();
            telemetry.addData("set     RPM: ", launchCurrentRPMTarget);
            telemetry.addData("Current RPM: ", launchCurrentRPM);
            telemetry.update();
        }
        intakeToElevatorThread.interrupt();
        launcherFeederThread.interrupt();
        wobbleGoalArmThread.interrupt();
        launcherThread.interrupt();
    }

    private void autoLaunchHighGoal() throws InterruptedException {
        // look for Tower VuMark
        int vIteration = 0;
        while ((!robot.vision.towerTargetScan()) && (vIteration < MAX_ITERATION)) {
            vIteration = vIteration + 1;
            sleep(50);
        }
        telemetry.addData("tower ", " first %d not found", vIteration);
        telemetry.update();
//        sleep(2000);

        // use Tower VuMark to correct robot position
        double towerOffsetX = towerOffsetTargetX;
        double towerOffsetY = towerOffsetTargetY;
        double towerOffsetAngle = towerOffsetTargetAngle;
        if (vIteration < MAX_ITERATION) {
            towerOffsetX = robot.vision.getTowerOffsetX();
            towerOffsetY = robot.vision.getTowerOffsetY();
            towerOffsetAngle = robot.vision.getTowerOffsetAngle();
            telemetry.addData("tower offset", " X %.1f Y %.1f Angle %.1f", towerOffsetX, towerOffsetY, towerOffsetAngle);
            telemetry.update();
//        sleep(2000);
            if (towerOffsetAngle > 90.0) {
                towerOffsetAngle = towerOffsetAngle - 180.0;
            }
            if (towerOffsetAngle < -90.0) {
                towerOffsetAngle = towerOffsetAngle + 180.0;
            }
            robot.drive.turnRobotByTick( -(towerOffsetAngle - towerOffsetTargetAngle));

            vIteration = 0;
            while ((!robot.vision.towerTargetScan()) && (vIteration < MAX_ITERATION)) {
                vIteration = vIteration + 1;
                sleep(50);
            }
            if (vIteration < MAX_ITERATION) {
                towerOffsetX = robot.vision.getTowerOffsetX();
                towerOffsetY = robot.vision.getTowerOffsetY();
                towerOffsetAngle = robot.vision.getTowerOffsetAngle();
                if (towerOffsetX > towerOffsetTargetX) {
                    robot.drive.moveLeft(towerOffsetX - towerOffsetTargetX);
                }
                else {
                    robot.drive.moveRight(towerOffsetTargetX - towerOffsetX);
                }
                if (towerOffsetY > towerOffsetTargetY) {
                    robot.drive.moveBackward(towerOffsetY - towerOffsetTargetY);
                }
                else {
                    robot.drive.moveForward(towerOffsetTargetY - towerOffsetY);
                }
            }

            // launch rings
            launchThreeRings();
        }
        else {
            telemetry.addLine("tower not found");
            telemetry.update();
        }

    }

    private void autoLaunchPowerShot() throws InterruptedException {
        robot.control.setLauncherTargetRPM(launchRPMPowerShot);
        // look for Tower VuMark
        int vIteration = 0;
        while ((!robot.vision.towerTargetScan()) && (vIteration < MAX_ITERATION)) {
            vIteration = vIteration + 1;
            sleep(50);
        }
        telemetry.addData("tower ", " first %d not found", vIteration);
        telemetry.update();
//        sleep(2000);

        // use Tower VuMark to correct robot position
        double towerOffsetX = towerOffsetTargetX;
        double towerOffsetY = towerOffsetTargetY;
        double towerOffsetAngle = towerOffsetTargetAngle;
        if (vIteration < MAX_ITERATION) {
            towerOffsetX = robot.vision.getTowerOffsetX();
            towerOffsetY = robot.vision.getTowerOffsetY();
            towerOffsetAngle = robot.vision.getTowerOffsetAngle();
            telemetry.addData("tower offset", " X %.1f Y %.1f Angle %.1f", towerOffsetX, towerOffsetY, towerOffsetAngle);
            telemetry.update();
//        sleep(2000);
            if (towerOffsetAngle > 90.0) {
                towerOffsetAngle = towerOffsetAngle - 180.0;
            }
            if (towerOffsetAngle < -90.0) {
                towerOffsetAngle = towerOffsetAngle + 180.0;
            }
            robot.drive.turnRobotByTick( -(towerOffsetAngle - towerOffsetTargetAngle));

            vIteration = 0;
            while ((!robot.vision.towerTargetScan()) && (vIteration < MAX_ITERATION)) {
                vIteration = vIteration + 1;
                sleep(50);
            }
            if (vIteration < MAX_ITERATION) {
                towerOffsetX = robot.vision.getTowerOffsetX();
                towerOffsetY = robot.vision.getTowerOffsetY();
                towerOffsetAngle = robot.vision.getTowerOffsetAngle();
                if (towerOffsetY > towerOffsetTargetY) {
                    robot.drive.moveBackward(towerOffsetY - towerOffsetTargetY);
                }
                else {
                    robot.drive.moveForward(towerOffsetTargetY - towerOffsetY);
                }
            }

            robot.drive.moveRight_odometry(launchPosX*mmPerInch - (towerOffsetX - towerOffsetTargetX));

            // scoop rings to the back
            robot.control.closeIntakeToElevator();
            sleep(600);
            robot.control.openIntakeToElevator();
            sleep(150);

            // raise elevator
            robot.control.moveElevator(1);
            sleep(800);

            // launch first ring
            robot.control.launchOneRing();
//        sleep(1000);
            robot.drive.moveRight_odometry(7.5*mmPerInch);

            // launch second ring
            robot.control.launchOneRing();
//        sleep(1000);
            robot.drive.moveRight_odometry(7.5*mmPerInch);

            // launch third ring
            robot.control.launchOneRing();

            // lower elevator to floor
            robot.control.moveElevatorToBottom();
        }
        else {
            telemetry.addLine("tower not found");
            telemetry.update();
        }
        robot.control.setLauncherTargetRPM(launchCurrentRPMTarget);
    }

    private void launchThreeRings() throws InterruptedException {

//        // start launcher
//        robot.control.setLaunchVelocity(-800.0); //722
//
//        // wait for launch motor to stabilize
//        sleep(5000);

        // scoop rings to the back
        robot.control.closeIntakeToElevator();
        sleep(600);
        robot.control.openIntakeToElevator();
        sleep(150);

        // raise elevator
        robot.control.moveElevator(1);
        sleep(800);

        // launch first ring
        robot.control.launchOneRing();
        sleep(800);

        // launch second ring
        robot.control.launchOneRing();
        sleep(800);

        // launch third ring
        robot.control.launchOneRing();

        // lower elevator to floor
        robot.control.moveElevatorToBottom();

    }



}