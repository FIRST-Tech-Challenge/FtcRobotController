package org.firstinspires.ftc.teamcode.opmodes.RedSideAutos;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.toolkit.PID.ShooterPIDController;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;

@Autonomous(name = "Red Auto", group = "opModes")
public class RedAuto extends UpliftAuto {
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
        robot.flickerSub.setFlickerPos(0.25);
        robot.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5, 0, 0, 25));
        robot.shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5, 0, 0, 25));
    }

    @Override
    public void body() throws InterruptedException {

        // extra safety measure to ensure that program does not run if stopped initially
        if(isStopRequested() || !opModeIsActive()) {
            return;
        }

        double startTime = System.currentTimeMillis();
        // set the initial position
        odom.setOdometryPosition(105.25, 8.5, 0);
        stack = robot.ringDetector.ringCount;
        robot.shooter1.setVelocity(robot.highGoalVelocity);
        robot.shooter2.setVelocity(robot.highGoalVelocity);
        robot.flickerSub.setFlickerPos(0.15);
        transferSub.raiseTransfer();


        if (stack == 0) {
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

        } else if (stack == 1) {
            driveSub.passThroughPosition(94, 24, 1, 0);
            driveSub.driveToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 1, 0);
            autoHighGoalShoot();

            // drop off first wobble
            driveSub.driveToPosition(116, 96, 1, 180);
            wobbleSub.dropOff();
            driveSub.driveToPosition(106, 80, 0.7, 180);

            // drive to pick up second wobble
            intakeSub.setIntakePower(1);
            driveSub.passThroughPosition(106, 44, 0.7, 180);
            robot.shooter1.setVelocity(robot.highGoalVelocity);
            robot.shooter2.setVelocity(robot.highGoalVelocity);
            driveSub.driveToPosition(115, 43, 0.7, 0, DriveSubsystem.COUNTER_CLOCKWISE);
            driveSub.driveToPosition(115, 32, 0.5, 0.5, 0, 0);
            wobbleSub.pickUp();
            intakeSub.setIntakePower(0);
            driveSub.driveToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 1, 0);
            transferSub.raiseTransfer();
            flickerSub.flickRing();
            robot.safeSleep(100);
            flickerSub.flickRing();
            transferSub.dropTransfer();
            shooterSub.setShooterPower(0);



            // drop off the second wobble goal
            driveSub.driveToPosition(110, 90, 1, 180);
            wobbleSub.dropOff();
            driveSub.driveToPosition(110, 80, 1, 180);


//            // jerk the intake to make sure ring is not stuck, while backing up from drop-off point
//            intakeSub.setIntakePower(-0.3);
//            intakeSub.setIntakePower(1);
//            driveSub.passThroughPosition(108, 80, 0.7, 180);
//            intakeSub.setIntakePower(0);
//
//            // shoot one powershot
//            robot.shooter1.setVelocity(robot.powerShotVelocity);
//            robot.shooter2.setVelocity(robot.powerShotVelocity);
//            driveSub.driveToPosition(DriveSubsystem.powershotShootingPt2.x, DriveSubsystem.powershotShootingPt2.y, 1, 0);
//            transferSub.raiseTransfer();
//            flickerSub.flickRing();
//            transferSub.dropTransfer();
//            shooterSub.setShooterPower(0);

            // park
            park();

        } else if (stack == 4) {
            driveSub.passThroughPosition(92, 24, 1, 0);
            driveSub.driveToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 1, 0);
            autoHighGoalShoot();

            // drive to drop off first wobble
            driveSub.driveToPosition(134, 122, 1, 180);
            wobbleSub.dropOff();
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

        } else {
            // shoot in high-goal
            driveSub.passThroughPosition(92, 24, 1, 0);
            driveSub.driveToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 1, 0);            autoHighGoalShoot();
            autoHighGoalShoot();

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
            robot.safeSleep(100);
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

}