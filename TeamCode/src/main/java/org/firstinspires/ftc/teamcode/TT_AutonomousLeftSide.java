package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import java.util.Locale;

@Autonomous(name = "Left Side Auto", group = "Techtonics")
//@Disabled

public class TT_AutonomousLeftSide extends LinearOpMode {
    TT_RobotHardware robot = new TT_RobotHardware(this);
    double autonomousPower = 1;
    double startingPoint = 0;   // just right of Center
    int dropCount = 0;

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_DROP_PREPARE_1,
        DRIVE_TO_DROP_1,
        DRIVE_TO_DROP_BACKUP,
        DRIVE_TO_BLOCK_1,
        DRIVE_TO_BLOCK_2,
        DRIVE_TO_BLOCK_3,
        DRIVE_TO_DROP_BLOCK,
        DRIVE_TO_PARKING,
    }

    // REMEMBER TO ADD 50 DISTANCE TO Y AXIS MOVING RIGHT
    int MAX_FORWARD_POSITION = -500;
    final Pose2D TARGET_SPECIMEN_DROP_PREPARE = new Pose2D(DistanceUnit.MM, calcXCoordinate(-300), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_BACKUP = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_1 = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), -1000, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_BLOCK_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), -1200, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_BLOCK_3 = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), -1000, AngleUnit.DEGREES, 225);
    final Pose2D TARGET_DROP_BLOCK = new Pose2D(DistanceUnit.MM, calcXCoordinate(-150), -1100, AngleUnit.DEGREES, 135);
    final Pose2D TARGET_50 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), 0, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {
        robot.init();
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_DROP_PREPARE_1;
                    robot.setLiftPosition(robot.liftHeightMax);
                    sleep(250);
                    break;
                case DRIVE_TO_DROP_PREPARE_1:
                    robot.setLiftPosition(robot.liftHeightMax);
                    autonomousPower = 1;
                    if ((driveToTarget(TARGET_SPECIMEN_DROP_PREPARE, 0, "Prepare for Drop of Specimen"))) {
                        stateMachine = StateMachine.DRIVE_TO_DROP_1;
                    }
                    break;
                case DRIVE_TO_DROP_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */

                    autonomousPower = 0.4;
                    if (driveToTarget(TARGET_SPECIMEN_DROP, 0.3, "Driving to Drop Position")) {
                        dropSpecimen();
                        stateMachine = StateMachine.DRIVE_TO_DROP_BACKUP;
                    }
                    break;
                case DRIVE_TO_DROP_BACKUP:
                    //drive to the second target
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_SPECIMEN_BACKUP, 0, "backup before turning")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_1;
                    }
                    break;
                case DRIVE_TO_BLOCK_1:
                    if (driveToTarget(TARGET_BLOCK_1, 2, "Position for first sample pickup")) {
                        stateMachine = StateMachine.DRIVE_TO_DROP_BLOCK;
                    }
                    break;
                case DRIVE_TO_BLOCK_2:
                    if (driveToTarget(TARGET_BLOCK_2, 2, "Position for second sample pickup")) {
                        stateMachine = StateMachine.DRIVE_TO_DROP_BLOCK;
                    }
                    break;
                case DRIVE_TO_BLOCK_3:
                    if (driveToTarget(TARGET_BLOCK_3, 2, "Position for third sample pickup")) {
                        stateMachine = StateMachine.DRIVE_TO_DROP_BLOCK;
                    }
                    break;
                case DRIVE_TO_DROP_BLOCK:
                    if (driveToTarget(TARGET_DROP_BLOCK, 2, "Moving to Drop Block")) {
                        dropSample();
                        if (dropCount == 1) {
                            stateMachine = StateMachine.DRIVE_TO_BLOCK_2;
                        } else if (dropCount == 2) {
                            stateMachine = StateMachine.DRIVE_TO_BLOCK_3;
                        } else if (dropCount == 3) {
                            stateMachine = StateMachine.DRIVE_TO_PARKING;
                        }
                    }
                    break;

                case DRIVE_TO_PARKING:
                    telemetry.addLine("Return to Start Position");
                    autonomousPower = .6;
                    if (driveToTarget(TARGET_50, 5, "Return to Start Position")) {
                        stateMachine = StateMachine.DRIVE_TO_PARKING;
                    }
                    break;
            }

            robot.setDrivePower();
            telemetry.addData("current state:", stateMachine);
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("Drop Count: %d", dropCount);
            telemetry.update();

        }
    }

    private void dropSample() {
        dropCount = dropCount + 1;
        robot.liftPowerMax = .8;
        robot.setLiftPosition(robot.liftHeightMax);
        sleep(300);
        robot.setLiftPosition(0);
        robot.liftPowerMax = 1;
    }

    private void dropSpecimen() {
        robot.liftPowerMax = .4;
        robot.setLiftPosition(robot.liftHeightMax - 600);
        sleep(300);
        robot.setLiftPosition(0);
        robot.liftPowerMax = 1;
    }
    private boolean driveToTarget(Pose2D target, double holdTime, String message) {
        telemetry.addLine(message);
        return (robot.nav.driveTo(robot.odo.getPosition(), target, autonomousPower, holdTime));
    }

    private double calcXCoordinate(double xChange) {
        return xChange + startingPoint;
    }
}
