package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import java.util.Locale;

@Autonomous(name = "Auto: Right Side", group = "Techtonics")
//@Disabled

public class TT_AutonomousRightSide extends LinearOpMode {
    TT_RobotHardware robot = new TT_RobotHardware(this);
    double autonomousPower = 1;
    double startingPoint = 0;   // just right of Center
    int dropCount = 0;
    boolean Four_Specimans = false;

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SPECIMEN_DROP_PREP,
        DRIVE_TO_SPECIMEN_DROP,
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
        DRIVE_TO_SPECIMEN_PICKUP_PREP,
        DRIVE_TO_SPECIMEN_PICKUP,
        DRIVE_TO_PARKING,
    }

    // REMEMBER TO ADD 50 DISTANCE TO Y AXIS MOVING RIGHT
    int MAX_FORWARD_POSITION = 1200;
    final Pose2D TARGET_SPECIMEN_DROP_PREPARE = new Pose2D(DistanceUnit.MM, calcXCoordinate(-300), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), -100, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_3 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), -200, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_4 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), -300, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-625), 500, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-(MAX_FORWARD_POSITION - 200)), 655, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-MAX_FORWARD_POSITION), 850, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 925, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_6 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-(MAX_FORWARD_POSITION - 500)), 900, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_7 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-MAX_FORWARD_POSITION), 1100, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_8 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 1150, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_9 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-(MAX_FORWARD_POSITION - 500)), 1100, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_10 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-MAX_FORWARD_POSITION), 1340, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_11 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 1340, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_PREPARE = new Pose2D(DistanceUnit.MM, calcXCoordinate(-200), 728, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_SPECIMEN_PICKUP = new Pose2D(DistanceUnit.MM, calcXCoordinate(0), 728, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_50 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), 0, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {
        runGeneralOpMode();
    }

    public void runGeneralOpMode() {
        telemetry.addData("auto:", "4 specimens %s",Four_Specimans);
        robot.init();
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;
        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        robot.extensionArm.setTargetPosition(.20);
        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP_PREP;
                    robot.setLiftPosition(robot.liftHeightSpecimenDrop);
                    break;
                case DRIVE_TO_SPECIMEN_DROP_PREP:
                    robot.setLiftPosition(robot.liftHeightSpecimenDrop);
                    autonomousPower = 1;
                    if ((driveToTarget(TARGET_SPECIMEN_DROP_PREPARE, 0, "Prepare for Drop of Specimen"))) {
                        stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP;
                    }
                    break;
                case DRIVE_TO_SPECIMEN_DROP:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */

                    autonomousPower = 0.4;

                    if (dropCount == 0) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP, 0.3, "Driving to Drop Position")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        }
                    } else if (dropCount == 1) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_2, 0.3, "Driving to Drop Position 2")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                        }
                    } else if (dropCount == 2) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_3, 0.3, "Driving to Drop Position 3")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                        }
                    } else if (dropCount == 3) {
                        if (driveToTarget(TARGET_SPECIMEN_DROP_4, 0.3, "Driving to Drop Position 4")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_2, 0, "Strafe right for moving forward")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (driveToTarget(TARGET_3, 0, "Forward to first block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_4, 0, "Move Right to first block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_5, 0, "Move back with first block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if (driveToTarget(TARGET_6, 0, "Forward to second block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;

                case DRIVE_TO_TARGET_7:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_7, 0, "Move Right to second block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_8, 0, "Move back with second block")) {
                        if (Four_Specimans) {
                            // Skip last block and instead go for putting on 3 more specimens.
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                        } else {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if (driveToTarget(TARGET_9, 0, "Forward to third block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                    }
                    break;

                case DRIVE_TO_TARGET_10:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_10, 0, "Move Right to third block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;

                case DRIVE_TO_TARGET_11:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_11, 0, "Move back with third block")) {
                        stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                    }
                    break;
                case DRIVE_TO_SPECIMEN_PICKUP_PREP:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_SPECIMEN_PREPARE, 0.3, "Prepare for Specimen Pickup")) {
                        stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP;
                    }
                    break;
                case DRIVE_TO_SPECIMEN_PICKUP:
                    autonomousPower = 0.4;
                    if (driveToTarget(TARGET_SPECIMEN_PICKUP, 0.1, "Pickup Specimen from wall")) {
                        if (Four_Specimans || (dropCount < 3)) {
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_DROP_PREP;
                        }
                        // ELSE this is the end of the program.   Don't drop any more and just park here.
                    }
                    break;

                case DRIVE_TO_PARKING:
                    telemetry.addLine("Return to Start Position");
                    autonomousPower = .4;
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
            robot.displayTelemetry();
        }
    }

    private void dropSpecimen() {
        dropCount = dropCount + 1;
        robot.liftPowerMax = .4;
        robot.setLiftPosition(robot.liftHeightSpecimenDrop - 600);
        sleep(250);
        robot.setLiftPosition(0);
        robot.liftPowerMax = .4;
    }

    private boolean driveToTarget(Pose2D target, double holdTime, String message) {
        telemetry.addLine(message);
        return (robot.nav.driveTo(robot.odo.getPosition(), target, autonomousPower, holdTime));
    }

    private double calcXCoordinate(double xChange) {
        return xChange + startingPoint;
    }
}
