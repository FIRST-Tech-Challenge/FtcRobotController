package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.TT_RobotHardware;

import java.util.Locale;

@Autonomous(name = "Left Side Auto", group = "Techtonics")

public class TT_AutonomousLeftSide extends LinearOpMode {
    TT_RobotHardware robot = new TT_RobotHardware(this);
    double autonomousPower = 1;
    double startingPoint = 0;   // just right of Center
    int dropCount = 0;
    boolean block_3 = false;
    boolean blockTransferred = false;

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_DROP_SPECIMEN_PREPARE,
        DRIVE_TO_DROP_SPECIMEN,
        DRIVE_TO_DROP_BACKUP,
        DRIVE_TO_BLOCK_1,
        DRIVE_TO_BLOCK_2,
        DRIVE_TO_BLOCK_3,
        DRIVE_TO_DROP_BLOCK,
        DRIVE_TO_PARK_PREP,
        DRIVE_TO_PARKING,
    }

    // REMEMBER TO ADD 50 DISTANCE TO Y AXIS MOVING RIGHT
    int MAX_FORWARD_POSITION = -430;
    final Pose2D TARGET_SPECIMEN_DROP_PREPARE = new Pose2D(DistanceUnit.MM, calcXCoordinate(-300), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_DROP = new Pose2D(DistanceUnit.MM, calcXCoordinate(-726), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_SPECIMEN_BACKUP = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), 0, AngleUnit.DEGREES, 0);
    final Pose2D TARGET_BLOCK_1 = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), -1005, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_BLOCK_2 = new Pose2D(DistanceUnit.MM, calcXCoordinate(MAX_FORWARD_POSITION), -1275, AngleUnit.DEGREES, 180);
    final Pose2D TARGET_BLOCK_3 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-570), -1200, AngleUnit.DEGREES, 225);
    final Pose2D TARGET_DROP_BLOCK = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), -1200, AngleUnit.DEGREES, 135);
    final Pose2D TARGET_PARK_PREP = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1268), -1000, AngleUnit.DEGREES, 90);
    final Pose2D TARGET_50 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-1268), -370, AngleUnit.DEGREES, 90);
    //final Pose2D TARGET_50 = new Pose2D(DistanceUnit.MM, calcXCoordinate(-100), 0, AngleUnit.DEGREES, 0);

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
                    stateMachine = StateMachine.DRIVE_TO_DROP_SPECIMEN_PREPARE;
                    robot.setLiftPosition(robot.liftHeightSpecimenDrop);
                    sleep(250);
                    break;
                case DRIVE_TO_DROP_SPECIMEN_PREPARE:
                    robot.setLiftPosition(robot.liftHeightSpecimenDrop);
                    autonomousPower = 1;
                    if ((driveToTarget(TARGET_SPECIMEN_DROP_PREPARE, 0, "Prepare for Drop of Specimen"))) {
                        stateMachine = StateMachine.DRIVE_TO_DROP_SPECIMEN;
                    }
                    break;
                case DRIVE_TO_DROP_SPECIMEN:
                    /*                    drive the robot to the first target                     */
                    autonomousPower = 0.4;
                    if (driveToTarget(TARGET_SPECIMEN_DROP, 0.3, "Driving to Drop Position")) {
                        dropSpecimen();
                        stateMachine = StateMachine.DRIVE_TO_DROP_BACKUP;
                    }
                    break;
                case DRIVE_TO_DROP_BACKUP:
                    autonomousPower = 1;
                    if (driveToTarget(TARGET_SPECIMEN_BACKUP, 0, "backup before turning")) {
                        stateMachine = StateMachine.DRIVE_TO_BLOCK_1;
                        robot.liftArm.setTargetPosition(robot.LIFT_ARM_DOWN);
                        robot.extensionArm.setTargetPosition(robot.EXT_ArmPickupReadyHeight);
                        robot.extensionGripper.setTargetPosition(.5);
                    }
                    break;
                case DRIVE_TO_BLOCK_1:
                    if (driveToTarget(TARGET_BLOCK_1, .5, "Position for first sample pickup")) {
                        pickupSample();
                        stateMachine = StateMachine.DRIVE_TO_DROP_BLOCK;
                    }
                    break;
                case DRIVE_TO_BLOCK_2:
                    if (driveToTarget(TARGET_BLOCK_2, .5, "Position for second sample pickup")) {
                        pickupSample();
                        stateMachine = StateMachine.DRIVE_TO_DROP_BLOCK;
                    }
                    break;
                case DRIVE_TO_BLOCK_3:
                    autonomousPower = .3;
                    robot.extensionSpin.setTargetPosition(.2);
                    if (driveToTarget(TARGET_BLOCK_3, .5, "Position for third sample pickup")) {
                        block_3 = true;
                        pickupSample();
                        stateMachine = StateMachine.DRIVE_TO_DROP_BLOCK;
                    }
                    break;
                case DRIVE_TO_DROP_BLOCK:
                    autonomousPower = 1;
                    if (blockTransferred == false) {
                        moveSampleToTransfer();
                    }
                    if (driveToTarget(TARGET_DROP_BLOCK, 1, "Moving to Drop Block")) {
                        dropSample();
                        if (dropCount == 1) {
                            stateMachine = StateMachine.DRIVE_TO_BLOCK_2;
                        } else if (dropCount == 2) {
                            stateMachine = StateMachine.DRIVE_TO_BLOCK_3;
                        } else if (dropCount == 3) {
                            stateMachine = StateMachine.DRIVE_TO_PARK_PREP;
                        }
                    }
                    break;

                case DRIVE_TO_PARK_PREP:
                    autonomousPower = 1;
                    robot.extension.setTargetPosition(1);
                    robot.extensionArm.setTargetPosition(robot.EXT_ArmDropHeight);
                    robot.extensionSpin.setTargetPosition(0.5);

                    if (driveToTarget(TARGET_PARK_PREP, 0, "Drive to Park Prep position")) {
                        stateMachine = StateMachine.DRIVE_TO_PARKING;
                    }
                    break;
                case DRIVE_TO_PARKING:
                    autonomousPower = .3;
                    if (driveToTarget(TARGET_50, 5, "Return to Start Position")) {
                        robot.extensionArm.setTargetPosition(robot.EXT_ArmMidHeight);
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

    private void pickupSample() {
        // Move Arm down to pickup block
        robot.extensionArm.setTargetPosition(1);
        sleep(500);
        // Grap Blaock
        robot.extensionGripper.setTargetPosition(0);
        sleep(100);
    }

    private void moveSampleToTransfer() {
        // Move extension out for transfer of block.
        // Move ext arm back for dropping cube into transfer box.
        // ensure rotation of gripper is at optimal position
        robot.extensionArm.setTargetPosition(robot.EXT_ArmDropHeight);
        robot.extensionSpin.setTargetPosition(.5);
        robot.extension.setTargetPosition(.7);
        blockTransferred = true;
    }

    private void dropSample() {
        dropCount = dropCount + 1;
        // Open Gripper - Drop block into Transfer box
        robot.extensionGripper.setTargetPosition(.5);
        sleep(300);
        // Lower arm to prepare for pickup and bring extension in
        // raise lift in preparation for dropping into scoring box
        robot.extensionArm.setTargetPosition(robot.EXT_ArmPickupReadyHeight);
        robot.extension.setTargetPosition(1);
        robot.liftPowerMax = 1;
        robot.setLiftPosition(robot.liftHeightMax);
        sleep(900);
        robot.liftArm.setTargetPosition(robot.LIFT_ARM_UP);
        sleep(850);
        robot.liftArm.setTargetPosition(robot.LIFT_ARM_DOWN);
        sleep(100);
        robot.liftPowerMax = 1;
        robot.setLiftPosition(0);
        robot.extensionArm.setTargetPosition(robot.EXT_ArmPickupReadyHeight);
        robot.extensionGripper.setTargetPosition(.5);
        blockTransferred = false;
    }

    private void dropSpecimen() {
        robot.liftPowerMax = .4;
        robot.setLiftPosition(robot.liftHeightSpecimenDrop - 600);
        sleep(250);
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
