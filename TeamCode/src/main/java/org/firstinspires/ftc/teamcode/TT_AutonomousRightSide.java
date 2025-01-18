package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import java.util.Locale;

@Autonomous(name = "Right Side Auto", group = "Techtonics")
//@Disabled

public class TT_AutonomousRightSide extends LinearOpMode {
    TT_RobotHardware robot = new TT_RobotHardware(this);
    double autonomousPower = 0.5;
    double startingPoint = 0;   // just right of Center

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        DRIVE_TO_TARGET_10,
        DRIVE_TO_TARGET_11,
        DRIVE_TO_TARGET_12,
        DRIVE_TO_TARGET_13,
        DRIVE_TO_TARGET_14,
        DRIVE_TO_TARGET_15,
    }
// REMEMBER TO ADD 50 DISTANCE TO Y AXIS MOVING RIGHT
    final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-745), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-600), 725, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1300), 725, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1300), 900, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), 900, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_6 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1300), 900, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_7 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1300), 1125, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_8 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), 1125, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_9 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1300), 1125, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_10 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1300), 1350, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_11 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), 1350, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_12 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-50), 0, AngleUnit.DEGREES, 0);
    @Override
    public void runOpMode() {
        robot.init();
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;
       robot.MaxPowerAdjustment=1.5;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    robot.setLiftPosition(robot.liftHeightMax);
                    sleep(250);
                    break;
                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */

                    robot.setLiftPosition(robot.liftHeightMax);
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_1, autonomousPower, 6)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        robot.setLiftPosition(1345);
                        sleep(500);
                        robot.setLiftPosition(0);

                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_2, autonomousPower, 0)) {
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_3, autonomousPower, 0)) {
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_4, autonomousPower, 0)) {
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_5, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                //case DRIVE_TO_TARGET_6:
                //    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_6, autonomousPower, 1)) {
                //       telemetry.addLine("There!");
                //       stateMachine = StateMachine.AT_TARGET;
                //   }
                //   break;
                case DRIVE_TO_TARGET_6:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_6, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;

                case DRIVE_TO_TARGET_7:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_7, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_8, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
               /* case DRIVE_TO_TARGET_9:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_9, autonomousPower, 5)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }

                    break;

                */
                case DRIVE_TO_TARGET_9:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_9, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                    }
                    break;

                case DRIVE_TO_TARGET_10:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_10, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;

                case DRIVE_TO_TARGET_11:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_11, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                    }
                    break;
                case DRIVE_TO_TARGET_12:
                    if (robot.nav.driveTo(robot.odo.getPosition(), TARGET_12, autonomousPower, 0)) {
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                    }
                    break;
            }


            robot.setDrivePower();
            telemetry.addData("current state:", stateMachine);
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();
            telemetry.update();

            //Pose2D pos = odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);

            //telemetry.update();
        }
    }

    private double calcXCoordinate(double xChange) {
        return xChange + startingPoint;
    }
}
