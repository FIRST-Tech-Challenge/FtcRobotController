package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import java.util.Locale;

@Autonomous(name = "Auto: Right Side v2", group = "Techtonics")
//@Disabled

public class TT_AutonomousRightSide_Verson2 extends LinearOpMode {
    TT_RobotHardware robot = new TT_RobotHardware(this);
    double autonomousPower = 1;
    double startingPoint = 0;   // just right of Center
    int dropCount = 0;
    boolean Four_Specimans = true;

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SPECIMEN_DROP_PREP,
        DRIVE_TO_SPECIMEN_DROP,
        DRIVE_TO_BLOCK_1_A,
        DRIVE_TO_BLOCK_1_B,
        DRIVE_TO_BLOCK_1_C,
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
    int MAX_FORWARD_POSITION = -950;

    final Pose2D TARGET_BLOCK_1_A = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_1_B = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 125, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_1_C = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 125, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_2_A = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 125, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_2_B = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 400, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_2_C = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 450, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_3_A = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 450, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_3_B = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 640, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_3_C = new Pose2D(DistanceUnit.MM, calcXCoordinate(-450), 640, AngleUnit.DEGREES, 0);

    final Pose2D TARGET_SPECIMEN_PREPARE = new Pose2D(DistanceUnit.MM, calcXCoordinate(-200), -128, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_SPECIMEN_PICKUP = new Pose2D(DistanceUnit.MM, calcXCoordinate(0), -128, AngleUnit.DEGREES, 180);

    final Pose2D TARGET_SPECIMEN_DROP_PREPARE = new Pose2D(DistanceUnit.MM, calcXCoordinate(-400), -800, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_1 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), -700, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), -775, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_3 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), -850, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP_4 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), -925, AngleUnit.DEGREES, 0);
    //final Pose2D TARGET_50 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), 0, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {
        runGeneralOpMode();
    }

    public void runGeneralOpMode() {
        telemetry.addData("auto:", "4 specimens %s",Four_Specimans);
        robot.init();
        StateMachine stateMachine;
        stateMachine = StateMachine.DRIVE_TO_BLOCK_1_A;
        // Wait for the game to start (driver presses START)

        //public double yawTolerance = 0.04; //0.0349066;
        robot.nav.yawTolerance = 0.1;
                waitForStart();
        resetRuntime();

        robot.extensionArm.setTargetPosition(.20);
        while (opModeIsActive()) {
            robot.odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_BLOCK_1_A;
                    //robot.setLiftPosition(robot.liftHeightSpecimenDrop);
                    break;
                case DRIVE_TO_BLOCK_1_A:
                    if (driveToTarget(TARGET_BLOCK_1_A, 0, "Forward to first block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_1_B;
                    }
                    break;
                case DRIVE_TO_BLOCK_1_B:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_BLOCK_1_B, 0, "Move Right to first block")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_1_C;
                    }
                    break;
                case DRIVE_TO_BLOCK_1_C:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_BLOCK_1_C, 0, "Move back with first block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if (driveToTarget(TARGET_BLOCK_2_A, 0, "Forward to second block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;

                case DRIVE_TO_TARGET_7:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_BLOCK_2_B, 0, "Move Right to second block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_BLOCK_2_C, 0, "Move back with second block")) {
                        if (Four_Specimans) {
                            // Skip last block and instead go for putting on 3 more specimens.
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
                        } else {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if (driveToTarget(TARGET_BLOCK_3_A, 0, "Forward to third block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                    }
                    break;

                case DRIVE_TO_TARGET_10:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_BLOCK_3_B, 0, "Move Right to third block")) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;

                case DRIVE_TO_TARGET_11:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_BLOCK_3_C, 0, "Move back with third block")) {
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
                        if (driveToTarget(TARGET_SPECIMEN_DROP_1, 0.3, "Driving to Drop Position")) {
                            dropSpecimen();
                            stateMachine = StateMachine.DRIVE_TO_SPECIMEN_PICKUP_PREP;
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
