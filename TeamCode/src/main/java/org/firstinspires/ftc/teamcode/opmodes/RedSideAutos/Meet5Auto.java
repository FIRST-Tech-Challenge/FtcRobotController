package org.firstinspires.ftc.teamcode.opmodes.RedSideAutos;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;
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
        intakeSub.initStick();
        transferSub.initTransferPos();
        intakeSub.initRoller();
        wobbleSub.closeWobble();
        wobbleSub.highWobble();
        intakeSub.initSweeper();
        flickerSub.setFlickerOut();
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
        shooterSub.setShooterVelocity(robot.autoHighGoalVelocity);
        transferSub.autoRaiseTransfer();

        intakeSub.dropRoller();
        intakeSub.dropSweeper();

        if(stack == 4) {

            // shoot
            shooterSub.setShooterVelocity(robot.autoHighGoalVelocity);
            driveSub.passThroughPosition(109, 24, 1, 0);
            driveSub.driveToPosition(109, 48, 0.4, 0);
            driveSub.turnTo(0,0.3, DriveSubsystem.QUICKEST_DIRECTION);
            autoHighGoalShoot();

            // intake four stack, if you were able to shoot all 3 initial rings
            if(robot.shotCount >= 3) {
                while(robot.transferState != UpliftRobot.TransferState.DOWN && opModeIsActive()) {
                    robot.safeSleep(5);
                }
                intakeSub.setIntakePower(1);
                double shootingActualY = robot.worldY;
                driveSub.driveToPosition(109, shootingActualY + 1.5, 0.3, 1, 0, DriveSubsystem.QUICKEST_DIRECTION);
                robot.safeSleep(1200);
                intakeSub.setIntakePower(0);

                // shoot second set of 3
                intakeSub.setIntakePower(-1);
                robot.safeSleep(200);
                transferSub.autoRaiseTransfer();
                intakeSub.liftRoller();
                shooterSub.setShooterVelocity(robot.autoHighGoalVelocity);
                driveSub.driveToPosition(109, 48, 0.7, 0);
                driveSub.turnTo(0,0.3, DriveSubsystem.QUICKEST_DIRECTION);
                intakeSub.setIntakePower(0);
                autoHighGoalShoot();

//                if(robot.shotCount >= 4) {
//                    // pick up last ring
//                    while (robot.transferState != UpliftRobot.TransferState.DOWN && opModeIsActive()) {
//                        robot.safeSleep(5);
//                    }
//                    shooterSub.setShooterVelocity(robot.autoHighGoalVelocity);
//                    intakeSub.dropRoller();
//                    intakeSub.setIntakePower(1);
//                    driveSub.driveToPosition(109, shootingActualY + 2.5, 0.3, 1, 0, DriveSubsystem.QUICKEST_DIRECTION);
//                    robot.safeSleep(400);
//                    intakeSub.setIntakePower(0);
//
//                    // shoot last ring
//                    intakeSub.setIntakePower(-1);
//                    robot.safeSleep(100);
//                    transferSub.autoRaiseTransfer();
//                    intakeSub.liftRoller();
//                    driveSub.driveToPosition(109, 48, 0.7, 0);
//                    driveSub.turnTo(0,0.3, DriveSubsystem.QUICKEST_DIRECTION);
//                    intakeSub.setIntakePower(0);
//                    while(robot.transferState != UpliftRobot.TransferState.UP && opModeIsActive()) {
//                        Utils.sleep(5);
//                    }
//                    flickerSub.flickRing();
//                    if(robot.shotCount <= 6) {
//                        flickerSub.flickRing();
//                    }
//                    transferSub.autoDropTransfer();
//                    shooterSub.setShooterPower(0);
//                }
            }

            // drive to drop off first wobble
            wobbleSub.setWobblePosition(0.4);
            driveSub.passThroughPosition(120, 102, 1, -135);
            driveSub.driveToPosition(128,120, 0.6, -135);
            wobbleSub.dropOff();
            driveSub.passThroughPosition(122, 114, 1, -135);

            // drive to pick up second wobble
            getSecondWobble();

            // drop off second wobble
            driveSub.passThroughPosition(120, 102, 1, -135);
            driveSub.driveToPosition(124,116, 0.6, -135);
            wobbleSub.dropOff();
            driveSub.driveToPosition(96, 88, 1, -135);

            //park
            intakeSub.dropRoller();
            driveSub.driveToPosition(94, 84, 0.5, 0);

        } else if(stack == 1) {

            // shoot
            shooterSub.setShooterVelocity(robot.autoHighGoalVelocity);
            driveSub.driveToPosition(107, 48, 0.7, 0);
            wobbleSub.setWobblePosition(0.85);
            wobbleSub.setWobblePosition(0.2);
            driveSub.turnTo(0,0.3, DriveSubsystem.QUICKEST_DIRECTION);
            intakeSub.liftRoller();
            autoHighGoalShoot();
            intakeSub.dropRoller();

            if(robot.shotCount >= 1) {
                // intake single stack
                while (robot.transferState != UpliftRobot.TransferState.DOWN && opModeIsActive()) {
                    Utils.sleep(5);
                }
                intakeSub.setIntakePower(1);
                driveSub.driveToPosition(110, 56, 0.25, 0);
                robot.safeSleep(750);
                intakeSub.setIntakePower(0);
                intakeSub.liftRoller();

                // shoot the 1 ring
                transferSub.autoRaiseTransfer();
                shooterSub.setShooterVelocity(robot.autoHighGoalVelocity);
                driveSub.driveToPosition(110, 48, 0.5, 0);
                for(int i = 0; i < 4 - robot.shotCount; i++) {
                    flickerSub.flickRing();
                }
                shooterSub.setShooterPower(0);
                transferSub.autoDropTransfer();
            }

            // drop off first wobble
            driveSub.driveToPosition(120, 94, 1, 180);
            driveSub.turnTo(180, 0.5, DriveSubsystem.QUICKEST_DIRECTION);
            driveSub.driveToPosition(120, 94, 0.6, 180);
            wobbleSub.dropOff();

            // pick up second wobble goal
            getSecondWobble();

            // drop off the second wobble goal
            driveSub.passThroughPosition(114, 80, 0.85, 180, DriveSubsystem.QUICKEST_DIRECTION, 6);
            driveSub.driveToPosition(114, 90, 0.7, 180);
            driveSub.turnTo(180, 0.5, DriveSubsystem.QUICKEST_DIRECTION);
            driveSub.driveToPosition(114, 90, 0.6, 180);
            wobbleSub.dropOff();

            // park
            park();
            intakeSub.dropRoller();

        } else {        // either 0 rings, or a problem with detection (-1)
            shooterSub.setShooterVelocity(robot.autoHighGoalVelocity);
            driveSub.driveToPosition(108, 48, 0.7, 0);
            autoHighGoalShoot();

            // drop off first wobble
            wobbleSub.setWobblePosition(0.2);
            driveSub.passThroughPosition(110, 54, 1, -135, DriveSubsystem.CLOCKWISE, 4);
            driveSub.driveToPosition(126, 70, 0.7, -135);
            driveSub.turnTo(-135, 0.5, DriveSubsystem.QUICKEST_DIRECTION);
            wobbleSub.dropOff();
            driveSub.driveToPosition(122, 66, 0.7, -135);

            // go to pick up second wobble
            getSecondWobble();

            // go to drop off second wobble
            driveSub.passThroughPosition(108, 54, 1, -135, DriveSubsystem.CLOCKWISE, 4);
            driveSub.driveToPosition(120, 66, 0.7, -135);
            driveSub.turnTo(-135, 0.5, DriveSubsystem.QUICKEST_DIRECTION);
            wobbleSub.dropOff();
            driveSub.driveToPosition(114, 62, 0.7,-135);

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
        driveSub.passThroughPosition(114, 56, 0.65, 0, DriveSubsystem.COUNTER_CLOCKWISE, 6);
        driveSub.turnTo(0, 0.75, DriveSubsystem.COUNTER_CLOCKWISE);
        driveSub.driveToPosition(114, 37, 0.3, 1, 0, 0);
        wobbleSub.pickUp();
    }

    public void autoHighGoalShoot() {
        while(robot.transferState != UpliftRobot.TransferState.UP && opModeIsActive()) {
            Utils.sleep(5);
        }
        robot.safeSleep(750);
        for(int i = 0; i < 3; i++) {
            double initialTime = System.currentTimeMillis();
            while(!robot.velocityData.isAutoHighGoalReady() && (System.currentTimeMillis() - initialTime) < 2000 && !robot.operatorCancel && opModeIsActive()) {
                robot.safeSleep(1);
            }
            flickerSub.flickRing();
            Log.i("Shot", (System.currentTimeMillis() - initialTime) + "     " + robot.shooter1Vel);
        }
        if(robot.shotCount == 2) {
            flickerSub.flickRing();
            Log.i("Shot extra", robot.shooter1Vel + "");
        }
        shooterSub.setShooterPower(0);
        transferSub.autoDropTransfer();
    }

//    public void fourStackShoot() {
//        double initialTime = System.currentTimeMillis();
//        while(robot.shooter1Vel < 2300 && (System.currentTimeMillis() - initialTime) < 2300 && opModeIsActive()) {
//            robot.safeSleep(1);
//        }
//
//        flickerSub.flickRing();
//        robot.safeSleep(100);
//        flickerSub.flickRing();
//        robot.safeSleep(100);
//        flickerSub.flickRing();
//
//        transferSub.autoDropTransfer();
//    }

}
