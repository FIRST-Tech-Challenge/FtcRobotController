package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

@Autonomous(name="Right Side Auto", group="Techtonics")
//@Disabled

public class TT_AutonomousRightSide extends LinearOpMode {
    TT_RobotHardware robot = new TT_RobotHardware(this);

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,500,20,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 500, -20, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,500,-500, AngleUnit.DEGREES,-90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 100, -500, AngleUnit.DEGREES, 90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 100, 0, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() {

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;


        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_1, 0.7, 0)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_2, 0.7, 1)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(robot.nav.driveTo(robot.odo.getPosition(), TARGET_3, 0.7, 3)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(robot.nav.driveTo(robot.odo.getPosition(),TARGET_4,0.7,1)){
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(robot.nav.driveTo(robot.odo.getPosition(),TARGET_5,0.7,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
            }
            robot.setDrivePower();
            telemetry.addData("current state:",stateMachine);

            //Pose2D pos = odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);

            //telemetry.update();

        }
    }}
