package org.firstinspires.ftc.teamcode.opmodes.RedSideAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;

@Autonomous(name = "Meet 5 Powershot Auto", group = "opModes")
public class Meet5PowershotAuto extends UpliftAuto {
    UpliftRobot robot;
    WobbleSubsystem wobbleSub;
    DriveSubsystem driveSub;
    IntakeSubsystem intakeSub;
    ShooterSubsystem shooterSub;
    TransferSubsystem transferSub;
    FlickerSubsystem flickerSub;
    Odometry odom;
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
    }

    @Override
    public void initAction() {
        wobbleSub.closeWobble();
        wobbleSub.highWobble();
    }

    @Override
    public void body() throws InterruptedException {
        double startTime = System.currentTimeMillis();
        // set the initial position
        odom.setOdometryPosition(105.25, 8.5, 0);
        stack = robot.ringDetector.ringCount;
        robot.shooter1.setVelocity(robot.powerShotVelocity);
        robot.shooter2.setVelocity(robot.powerShotVelocity);
        transferSub.raiseTransfer();

        if (stack == 0) {
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt1.x, DriveSubsystem.powershotShootingPt1.y, 1, 0);
            autoPowerShotShoot();
            // drop off first wobble
            driveSub.driveToPosition(130, 74, 1, 180, DriveSubsystem.CLOCKWISE);
            wobbleSub.dropOff();
            driveSub.driveToPosition(130, 70, 1, 180);

            // go to pick up second wobble
            wobbleSub.highWobble();
            getSecondWobble();

            // go to drop off second wobble
            driveSub.driveToPosition(132, 64, 1, 180, DriveSubsystem.CLOCKWISE);
            wobbleSub.dropOff();
            driveSub.driveToPosition(132, 60, 1,180);

            // park
            park();

        } else if (stack == 1) {
            driveSub.passThroughPosition(94, 24, 1, 0);
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt1.x, DriveSubsystem.powershotShootingPt1.y, 1, 0);
            autoPowerShotShoot();

            // drop off first wobble
            driveSub.driveToPosition(114, 100, 1, 180);
            wobbleSub.dropOff();
            driveSub.passThroughPosition(114, 90, 0.7, 180);

            // drive to pick up second wobble
            driveSub.passThroughPosition(130, 70, 0.7, 180);
            getSecondWobble();

            // intake the 1 ring from the stack while going to drop off the second wobble
            intakeSub.setIntakePower(1);
            driveSub.passThroughPosition(108, 20, 1, 0);
            driveSub.passThroughPosition(108, 60, 0.5, 0);

            // drop off the second wobble goal
            driveSub.driveToPosition(108, 90, 1, 180);
            intakeSub.setIntakePower(0);
            wobbleSub.dropOff();

            // jerk the intake to make sure ring is not stuck, while backing up from drop-off point
            intakeSub.setIntakePower(-0.3);
            intakeSub.setIntakePower(1);
            driveSub.passThroughPosition(108, 80, 0.7, 180);
            intakeSub.setIntakePower(0);

            // shoot one high-goal
            robot.shooter1.setVelocity(robot.highGoalVelocity);
            robot.shooter2.setVelocity(robot.highGoalVelocity);
            driveSub.driveToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 1, 0);
            transferSub.raiseTransfer();
            flickerSub.flickRing();
            transferSub.dropTransfer();
            shooterSub.setShooterPower(0);

            // park
            park();

        } else if (stack == 4) {
            driveSub.passThroughPosition(92, 24, 1, 0);
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt1.x, DriveSubsystem.powershotShootingPt1.y, 1, 0);
            autoPowerShotShoot();

            // drive to drop off first wobble
            driveSub.driveToPosition(134, 122, 1, 180);
            wobbleSub.dropOff();
            driveSub.driveToPosition(134, 112, 1, 180);
            wobbleSub.highWobble();

            // drive to pick up second wobble
            wobbleSub.highWobble();
            getSecondWobble();

            // drop off second wobble
            driveSub.driveToPosition(138, 116, 1, 180);
            wobbleSub.dropOff();
            driveSub.driveToPosition(138, 106, 1, 180);

            // put wobble mechanism up for teleop
            wobbleSub.highWobble();

            // park by driving forward to the line
            driveSub.driveToPosition(138, 84, 1,180);
            driveSub.stopMotors();

        } else {
            // shoot in high-goal
            driveSub.passThroughPosition(92, 24, 1, 0);
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt1.x, DriveSubsystem.powershotShootingPt1.y, 1, 0);
            autoPowerShotShoot();

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
        driveSub.driveToPosition(117.25, 42, 0.7, 0, DriveSubsystem.COUNTER_CLOCKWISE);
        driveSub.driveToPosition(117.25, 34, 1, 0.5, 0, 0);
        wobbleSub.pickUp();
    }

    public void autoPowerShotShoot() {
        double initialTime = System.currentTimeMillis();
        while(!robot.velocityData.isPowerShotShooterReady() && (System.currentTimeMillis() - initialTime) < 2000 && opModeIsActive()) {
            robot.safeSleep(1);
        }
        if(System.currentTimeMillis() - initialTime > 2000) {
            robot.flickerSub.flickRing();
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt2.x, DriveSubsystem.powershotShootingPt2.y, 1, 0);
            robot.flickerSub.flickRing();
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt3.x, DriveSubsystem.powershotShootingPt3.y, 1, 0);
            robot.flickerSub.flickRing();
        } else {
            robot.flickerSub.flickRing();
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt2.x, DriveSubsystem.powershotShootingPt2.y, 1, 0);
            while(!robot.velocityData.isPowerShotShooterReady() && (System.currentTimeMillis() - initialTime) < 2000 && opModeIsActive()) {
                robot.safeSleep(1);
            }
            robot.flickerSub.flickRing();
            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt3.x, DriveSubsystem.powershotShootingPt2.y, 1, 0);
        }
        robot.shooterSub.setShooterPower(0);
        robot.transferSub.dropTransfer();
    }

}