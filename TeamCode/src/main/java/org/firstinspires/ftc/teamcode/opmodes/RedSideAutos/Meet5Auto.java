package org.firstinspires.ftc.teamcode.opmodes.RedSideAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.background.AutoTimeout;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;

@Autonomous(name = "Meet 5 Auto", group = "opModes")
public class Meet5Auto extends UpliftAuto {
    UpliftRobot robot;
    WobbleSubsystem wobbleSub;
    DriveSubsystem driveSub;
    IntakeSubsystem intakeSub;
    ShooterSubsystem shooterSub;
    TransferSubsystem transferSub;
    FlickerSubsystem flickerSub;
    Odometry odom;
    AutoTimeout autoTimeout;
    int stack;
    RingDetector detector;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        wobbleSub = robot.wobbleSub;
        driveSub = robot.driveSub;
        shooterSub = robot.shooterSub;
        odom = robot.odometry;
        intakeSub = robot.intakeSub;
        transferSub = robot.transferSub;
        flickerSub = robot.flickerSub;
        detector = robot.ringDetector;
        autoTimeout = new AutoTimeout(robot);
        autoTimeout.enable();
    }

    @Override
    public void initAction() {
        intakeSub.initRoller();
        wobbleSub.closeWobble();
        wobbleSub.highWobble();
        transferSub.initTransferPos();
        intakeSub.initStick();
        intakeSub.initSweeper();
    }

    @Override
    public void body() throws InterruptedException {

        // extra safety measure to ensure that program does not run if stopped initially
        if(isStopRequested() || !opModeIsActive()) {
            return;
        }

        // set the initial position, ring stack count, and prepare shooter/transfer/intake
        odom.setOdometryPosition(105.25, 8.5, 0);
        stack = robot.ringDetector.ringCount;
        shooterSub.setShooterVelocity(robot.highGoalVelocity);
        transferSub.raiseTransfer();

        if(stack == 4) {
            // shoot
            shooterSub.setShooterVelocity(2300);
            driveSub.driveToPosition(105.25, 12, 0.5, 0);
            fourStackShoot();

            // intake four stack
            driveSub.driveToPosition(108, 20, 1, 0);
            driveSub.driveToPosition(108, 16, 1, 0);
            intakeSub.setIntakePower(1);
            driveSub.driveToPosition(108, 24, 0.5, 0);
            intakeSub.setIntakePower(0);

            // shoot second set of 3
            transferSub.raiseTransfer();
            driveSub.driveToPosition(105.25, 12, 0.5, 0);
            fourStackShoot();

            // intake last ring
            intakeSub.setIntakePower(1);
            driveSub.driveToPosition(108, 28, 0.5, 0);
            intakeSub.setIntakePower(0);

            // shoot last ring
            transferSub.raiseTransfer();
            driveSub.driveToPosition(105.25, 12, 0.5, 0);
            flickerSub.flickRing();

            // drive to drop off first wobble
            driveSub.driveToPosition(134, 122, 1, 180);
            wobbleSub.dropWobble();
            robot.safeSleep(500);
            wobbleSub.openWobble();
            driveSub.driveToPosition(134, 112, 1, 180);

            // drive to pick up second wobble
            driveSub.driveToPosition(115, 43, 0.7, 0, DriveSubsystem.COUNTER_CLOCKWISE);
            driveSub.driveToPosition(115, 32, 0.5, 0.5, 0, 0);
            wobbleSub.pickUp();

            // drop off second wobble
            driveSub.driveToPosition(134, 116, 1, -135);
            wobbleSub.dropOff();
            driveSub.driveToPosition(124, 106, 1, -135);

            // put wobble mechanism up for teleop
            wobbleSub.highWobble();

            // park by driving forward to the line
            park();

        } else if(stack == 1) {
            shooterSub.setShooterVelocity(2300);
            driveSub.driveToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 1, 0);
            fourStackShoot();
            intakeSub.setIntakePower(1);
            driveSub.driveToPosition(108, 28, 0.5, 0);
            intakeSub.setIntakePower(0);

            // shoot last ring
            transferSub.raiseTransfer();
            driveSub.driveToPosition(105.25, 12, 0.5, 0);
            flickerSub.flickRing();


            // drop off first wobble
            driveSub.driveToPosition(116, 96, 1, 180);
            wobbleSub.dropOff();
            driveSub.driveToPosition(106, 80, 0.7, 180);

            // turn on shooter and pick up second wobble goal
            driveSub.driveToPosition(115, 43, 0.7, 0, DriveSubsystem.COUNTER_CLOCKWISE);
            driveSub.driveToPosition(115, 32, 0.5, 0.5, 0, 0);
            wobbleSub.pickUp();

            // drop off the second wobble goal
            driveSub.driveToPosition(110, 90, 1, 180);
            wobbleSub.dropOff();
            driveSub.driveToPosition(110, 80, 1, 180);

            // park
            park();

        } else {        // either 0 rings, or a problem with detection (-1)
            driveSub.driveToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 1, 0);
            autoHighGoalShoot();
            // drop off first wobble
            driveSub.driveToPosition(130, 74, 1, 180, DriveSubsystem.CLOCKWISE);
            wobbleSub.dropOff();
            driveSub.driveToPosition(130, 70, 1, 180);

            // go to pick up second wobble
            getSecondWobble();

            // go to drop off second wobble
            driveSub.driveToPosition(132, 64, 1, 180, DriveSubsystem.CLOCKWISE);
            wobbleSub.dropOff();
            driveSub.driveToPosition(132, 60, 1,180);

            // park
            park();
        }

    }

    @Override
    public void exit() throws InterruptedException {
        driveSub.stopMotors();
        robot.writePositionToFiles();
    }

    public void park() {
        wobbleSub.highWobble();
        driveSub.driveToPosition(94, 84, 1, 0);
        driveSub.stopMotors();
    }

    public void getSecondWobble() {
        driveSub.driveToPosition(117, 43, 0.7, 0, DriveSubsystem.COUNTER_CLOCKWISE);
        driveSub.driveToPosition(117, 32, 0.5, 0.5, 0, 0);
        wobbleSub.pickUp();
    }

    public void autoHighGoalShoot() {
        double initialTime = System.currentTimeMillis();
        while(!robot.velocityData.isHighGoalShooterReady() && (System.currentTimeMillis() - initialTime) < 2000 && opModeIsActive()) {
            robot.safeSleep(1);
        }
        if(System.currentTimeMillis() - initialTime >= 2000) {
            flickerSub.flickRing();
            robot.safeSleep(100);
            flickerSub.flickRing();
            robot.safeSleep(100);
            flickerSub.flickRing();
        } else {
            for(int i = 0; i < 3 && opModeIsActive(); i++) {
                while (!robot.velocityData.isHighGoalShooterReady() && (System.currentTimeMillis() - initialTime) < 2000 && opModeIsActive()) {
                    robot.safeSleep(1);
                }
                flickerSub.flickRing();
            }
        }
        shooterSub.setShooterPower(0);
        transferSub.dropTransfer();
    }

    public void fourStackShoot() {
        double initialTime = System.currentTimeMillis();
        while(robot.shooter1Vel < 2300 && (System.currentTimeMillis() - initialTime) < 2300 && opModeIsActive()) {
            robot.safeSleep(1);
        }

        flickerSub.flickRing();
        robot.safeSleep(100);
        flickerSub.flickRing();
        robot.safeSleep(100);
        flickerSub.flickRing();

        transferSub.dropTransfer();
    }

}
