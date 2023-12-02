package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.datalogger.HydraObjDetDatalogger;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.subsystems.HydraArm;
import org.firstinspires.ftc.teamcode.subsystems.HydraDrive;
import org.firstinspires.ftc.teamcode.subsystems.HydraIntake;
import org.firstinspires.ftc.teamcode.subsystems.HydraObjectDetect;
import org.firstinspires.ftc.teamcode.subsystems.HydraPixelPalace;
import org.firstinspires.ftc.teamcode.types.HydraArmMovements;
import org.firstinspires.ftc.teamcode.types.HydraObjectLocations;
import org.firstinspires.ftc.teamcode.types.HydraPixelPalaceActions;
import org.firstinspires.ftc.teamcode.datalogger.HydraDriveDatalogger;

import java.util.List;

//@Autonomous(name = "HydrAutonJava", preselectTeleOp = "HyDrive")
public class HydrAuton extends LinearOpMode {
    protected int mWaitTimeAtRigging = 13000;
    protected IMU imu;
    protected HydraArm Arm;
    protected HydraDrive Drive;
    protected HydraPixelPalace PixelPalace;
    protected HydraIntake Intake;
    protected HydraObjectLocations ObjLoc;
    protected HydraObjectDetect ObjDet;
    protected ElapsedTime pixelDropTimer;
    protected ElapsedTime opModeTimer;
    protected int autonState;
    protected String modelFilename = "Blue_Prop.tflite";
    protected String mOpModeName = "HydrAuton";
    protected boolean setTrueForRed = false;
    protected boolean setTrueForRiggingOnRight = false;
    protected HydraOpMode mOp;
    protected MultipleTelemetry dashboard;
    protected final int cMaxObjectSearchTimeMs = 2000;
    protected final int cPixelDropRunTimeMs = 2000;
    protected final int cPixelFrontScoreRunTimeMs = 2000;
    protected final int cAutonAbortTimeMs = 27000;
    protected HydraDriveDatalogger mDriveLogger;
    protected HydraObjDetDatalogger mObjLogger;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // Initialize Local Variables
        autonState = 0;
        ObjLoc = HydraObjectLocations.ObjLocUnknown;
        pixelDropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        opModeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean autonAbort = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mDriveLogger = new HydraDriveDatalogger("drive-log-" + mOpModeName);
        mObjLogger = new HydraObjDetDatalogger("obj-log-" + mOpModeName);
        mOp = new HydraOpMode(telemetry, hardwareMap, mDriveLogger, mObjLogger);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        // Initialization Routines
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        Arm = new HydraArm(mOp);
        Drive = new HydraDrive(mOp);
        PixelPalace = new HydraPixelPalace(mOp);
        Intake = new HydraIntake(mOp);
        ObjDet = new HydraObjectDetect(mOp, modelFilename);
        // disable AprilTag detection until we need it
        ObjDet.SetAprilDetectEnabled(false);
        // print any telemetry that came from initialization of the subsystems
        mOp.mTelemetry.update();
        // manual caching mode
        for (LynxModule mod : hubs) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        while (!ObjDet.CameraIsReady()) {
            if (isStopRequested()) {
                break;
            }
        }
        // Wait for the match to begin.
        waitForStart();
        // Useful code to load pixels before we run. DISABLE FOR COMPETITION
        /*{
            while (opModeIsActive()) {
                if (PixelPalace.Start(HydraPixelPalaceActions.PixelPalaceFrontToBack,
                        HydraPixelPalaceActions.PixelPalaceFrontToBack, false) == 3) {
                    break;
                }
                Intake.StartIn();
            }
            Intake.Stop();
        }*/
        // Use this timer for time elapsed in the opmode. Do not reset it
        opModeTimer.reset();
        int loops = 0;
        if (mDriveLogger != null) {
            mDriveLogger.loops.set(loops);
            mDriveLogger.writeLine();
        }
        if (mObjLogger != null) {
            mObjLogger.loops.set(loops);
            mObjLogger.writeLine();
        }
        // Find the object so we can drive to it
        while (opModeIsActive()) {
            // Run tensorflow to see if we can find the object
            ObjLoc = ObjDet.GetObjectLocation(setTrueForRed);
            // UNCOMMENT THIS TO HARDCODE THE OBJECT LOCATION
            // ObjLoc = HydraObjectLocations.ObjLocCenterSpike;
            // Push telemetry to the Driver Station.
            telemetry.update();
            ++loops;
            if (mObjLogger != null) {
                mObjLogger.loops.set(loops);
            }
            // If we found the device or if we have been searching for a long time, leave
            if (ObjLoc != HydraObjectLocations.ObjLocUnknown ||
                    opModeTimer.milliseconds() >= cMaxObjectSearchTimeMs) {
                break;
            }
            sleep(20);
        }
        // If we did not find it, we have no choice but to assume that it was in the position we can't see
        if (ObjLoc == HydraObjectLocations.ObjLocUnknown) {
            if (setTrueForRed) {
                ObjLoc = HydraObjectLocations.ObjLocRedRightSpike;
            }
            else {
                ObjLoc = HydraObjectLocations.ObjLocBlueRightSpike;
            }
        }
        // enable AprilTag detection
        ObjDet.SetAprilDetectEnabled(true);
        // disable object detection now to save CPU
        ObjDet.SetObjDetectEnabled(false);
        // Run the proper auton for the object location
        while (opModeIsActive()) {
            telemetry.addData("Location", ObjLoc);
            // The auton returns true when it's done
            if (RunAuton()) {
                break;
            }
            // If we have been running for too long, we need to stop and get ready for the drive period
            if (opModeTimer.milliseconds() >= cAutonAbortTimeMs) {
                autonAbort = true;
                // this is the initial state for bringing the Arm home in all autons
                autonState = 400;
                ArmToHome();
                break;
            }
            telemetry.addData("State", autonState);
            telemetry.update();
            ++loops;
            if (mDriveLogger != null) {
                mDriveLogger.loops.set(loops);
                mDriveLogger.state.set(autonState);
            }
            if (mObjLogger != null) {
                mObjLogger.loops.set(loops);
                mObjLogger.state.set(autonState);
            }
            // Share the CPU.
            sleep(20);
        }
        // if we had to abort, we allow the arm to return home
        while (autonAbort && opModeIsActive()) {
            ArmToHome();
            if (autonState >= 500) {
                break;
            }
            telemetry.addData("State", autonState);
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }
        // wait for the opmode to time out
        while (opModeIsActive() && opModeTimer.milliseconds() < 30000) {
            sleep(20);
        }
    }

    /**
     * This function runs the auton. This base class has no implementation of its own
     * @return true when the auton is completed
     */
    protected boolean RunAuton() {
        return true;
    }
    /**
     * Logs the current auton state to the telemetry for debugging
     */
    protected void BadState() {
        telemetry.addData("InvalidState", autonState);
    }

    /**
     * Drive the robot to the detected object location [ObjLoc]
     * This function is used for all 4 auton op modes
     * @param flipWhenRiggingIsRight is set to true if the rigging is to the robots right at start
     * @return false iff there is a problem
     */
    protected boolean AutonDriveToSpike(boolean flipWhenRiggingIsRight) {
        // multiply strafes and rotates by -1 based on the starting orientation
        int flip = 1;
        if (flipWhenRiggingIsRight) {
            flip = -1;
        }
        switch (autonState) {
            case 0:
                // Jump to the correct state based on the location
                switch (ObjLoc) {
                    case ObjLocBlueRightSpike:
                    case ObjLocRedRightSpike:
                        if (flipWhenRiggingIsRight) {
                            autonState = 30;
                        }
                        else {
                            autonState = 10;
                        }
                        break;
                    case ObjLocBlueCenterSpike:
                    case ObjLocRedCenterSpike:
                        autonState = 20;
                        break;
                    case ObjLocBlueLeftSpike:
                    case ObjLocRedLeftSpike:
                        if (flipWhenRiggingIsRight) {
                            autonState = 10;
                        }
                        else {
                            autonState = 30;
                        }
                        break;
                    default:
                        return false;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 10:
                // LEFT RIGGING RIGHT SPIKE
                // RIGHT RIGGING LEFT SPIKE
                Drive.Start(16, 13 * flip, 0);
                autonState = 99;
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 20:
                // CENTER SPIKE
                Drive.Start(28, 20 * flip, 0);
                autonState += 1;
                break;
            case 21:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(8, 0, 0);
                    autonState += 1;
                }
                break;
            case 22:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(0, 0, -20 * flip);
                    autonState = 99;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 30:
                // LEFT RIGGING LEFT SPIKE
                // RIGHT RIGGING RIGHT SPIKE
                Drive.Start(30, 3 * flip, 0);
                autonState += 1;
                break;
            case 31:
                // LEFT RIGGING LEFT SPIKE
                // RIGHT RIGGING RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(0, 0, -20 * flip);
                    autonState = 99;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 99:
                // Wait for all driving to stop before moving on
                if (!Drive.Busy()) {
                    autonState = 100;
                }
                break;
            default:
                // Something unexpected happened
                return false;
        }
        return true;
    }

    /**
     * Drive the robot to the backdrop from the wing based on which spike we delivered at [ObjLoc]
     * This function is used for both wing or audience side auton op modes
     * @param flipWhenRed is set to true if we are running a red team auton
     * @return false iff there is a problem
     */
    protected boolean AutonDriveToBackdropFromWing(boolean flipWhenRed, boolean parkOnly) {
        // multiply strafes and rotates by -1 based on the starting orientation
        int flip = 1;
        if (flipWhenRed) {
            flip = -1;
        }
        switch (autonState) {
            case 200:
                switch (ObjLoc) {
                    // jump to the correct state based on the detected location
                    case ObjLocBlueLeftSpike:
                    case ObjLocRedRightSpike:
                        autonState = 230;
                        break;
                    case ObjLocBlueRightSpike:
                    case ObjLocRedLeftSpike:
                        autonState = 210;
                        break;
                    case ObjLocBlueCenterSpike:
                    case ObjLocRedCenterSpike:
                        autonState = 220;
                        break;
                    default:
                        return false;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 210:
                // BLUE RIGHT
                // RED LEFT
                Drive.Start(0, -12 * flip, 0);
                autonState += 1;
                break;
            case 211:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy()) {
                    Drive.Start(33, 0, 0);
                    autonState += 1;
                }
                break;
            case 212:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy()) {
                    Drive.Start(0, 0, -20 * flip);
                    autonState += 1;
                }
                break;
            case 213:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy() && opModeTimer.milliseconds() >= mWaitTimeAtRigging) {
                    if (parkOnly) {
                        Drive.Start(86, 0, 0);
                        autonState = 299;
                    }
                    else {
                        Drive.Start(73, 0, 0);
                        autonState += 1;
                    }
                }
                break;
            case 214:
                // BLUE RIGHT
                // RED LEFT
                if (!Drive.Busy()) {
                    Drive.Start(0, -24 * flip, 0);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 220:
                // CENTER
                if (!Drive.Busy()) {
                    Drive.Start(0, 18 * flip, 0);
                    autonState += 1;
                }
                break;
            case 221:
                // CENTER
                if (!Drive.Busy() && opModeTimer.milliseconds() >= mWaitTimeAtRigging) {
                    if (parkOnly) {
                        Drive.Start(103, 0, 0);
                        autonState = 299;
                    }
                    else {
                        Drive.Start(90, 0, 0);
                        autonState += 1;
                    }
                }
                break;
            case 222:
                // CENTER
                if (!Drive.Busy()) {
                    Drive.Start(0, -27 * flip, 0);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 230:
                // BLUE LEFT
                // RED RIGHT
                if (!Drive.Busy()) {
                    Drive.Start(0, 22 * flip, 0);
                    autonState += 1;
                }
                break;
            case 231:
                // BLUE LEFT
                // RED RIGHT
                if (!Drive.Busy() && opModeTimer.milliseconds() >= mWaitTimeAtRigging) {
                    if (parkOnly) {
                        Drive.Start(89, 0, 0);
                        autonState = 299;
                    }
                    else {
                        Drive.Start(76, 0, 0);
                        autonState += 1;
                    }
                }
                break;
            case 232:
                // BLUE LEFT
                // RED RIGHT
                if (!Drive.Busy()) {
                    Drive.Start(0, -29 * flip, 0);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 299:
                boolean drivecomplete = !Drive.Busy();
                boolean armcomplete = true;
                if (!parkOnly) {
                    armcomplete = Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                }
                if (drivecomplete && armcomplete) {
                    autonState = 300;
                }
                break;
            default:
                // something bad happened
                return false;
        }
        return true;
    }

    /**
     * Drive the robot to the backdrop from the backstage based on which spike we delivered at [ObjLoc]
     * This function is used for both backstage side auton op modes
     * @param flipForRed is set to true if we are running a red team auton
     * @return false iff there is a problem
     */
    protected boolean AutonDriveToBackdropFromBackstage(boolean flipForRed) {
        // multiply strafes and rotates by -1 based on the starting orientation
        int flip = 1;
        if (flipForRed) {
            flip = -1;
        }
        switch (autonState) {
            case 200:
                // jump to the correct state based on which spike we are at
                switch (ObjLoc) {
                    case ObjLocBlueLeftSpike:
                    case ObjLocRedRightSpike:
                        autonState = 210;
                        break;
                    case ObjLocBlueCenterSpike:
                    case ObjLocRedCenterSpike:
                        autonState = 220;
                        break;
                    case ObjLocBlueRightSpike:
                    case ObjLocRedLeftSpike:
                        autonState = 230;
                        break;
                    default:
                        return false;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 210:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-4, 0, 0);
                    autonState += 1;
                }
                break;
            case 211:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(0, 0, 20 * flip);
                    autonState += 1;
                }
                break;
            case 212:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-20, -10 * flip, 0);
                    Arm.RunAction(HydraArmMovements.ArmMoveToBack);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 220:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(0, 10 * flip, 0);
                    autonState += 1;
                }
                break;
            case 221:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-14, 0, 0);
                    Arm.RunAction(HydraArmMovements.ArmMoveToBack);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 230:
                // BLUE RIGHT SPIKE
                // RED LEFT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-28, 0, 0);
                    Arm.RunAction(HydraArmMovements.ArmMoveToBack);
                    autonState = 299;
                }
                break;
            ///////////////////////////////////////////////////////////////////////////////////////
            case 299:
                boolean drivecomplete = !Drive.Busy();
                boolean armcomplete = Arm.RunAction(HydraArmMovements.ArmMoveToBack);
                if (drivecomplete && armcomplete) {
                    autonState = 300;
                }
                break;
            default:
                // something bad happened
                return false;
        }
        return true;
    }

    /**
     * Score a pixel on the backdrop from the back of the robot
     * @return false iff something bad happens
     */
    protected boolean ScoreBack() {
        switch (autonState) {
            case 300:
                if (Arm.RunAction(HydraArmMovements.ArmMoveToBack)) {
                    autonState += 1;
                }
                break;
            case 301:
                if (!Drive.Busy()) {
                    Drive.Start(-6, 0, 0);
                    autonState += 1;
                }
                break;
            case 302:
                if (!Drive.Busy()) {
                    PixelPalace.Start(HydraPixelPalaceActions.PixelPalaceBackToFront,
                            HydraPixelPalaceActions.PixelPalaceBackToFront, true);
                    pixelDropTimer.reset();
                    autonState += 1;
                }
                break;
            case 303:
                if (pixelDropTimer.milliseconds() >= cPixelFrontScoreRunTimeMs) {
                    PixelPalace.Stop();
                    Drive.Start(4, 0, 0);
                    autonState = 400;
                }
                break;
            default:
                return false;
        }
        return true;
    }

    /**
     * Score a pixel on the backdrop from the front of the robot
     * @return false iff something bad happens
     */
    protected boolean ScoreFront() {
        switch (autonState) {
            case 300:
                if (Arm.RunAction(HydraArmMovements.ArmMoveToFront)) {
                    autonState += 1;
                }
                break;
            case 301:
                if (!Drive.Busy()) {
                    Drive.Start(7, 0, 0);
                    autonState += 1;
                }
                break;
            case 302:
                if (!Drive.Busy()) {
                    PixelPalace.Start(HydraPixelPalaceActions.PixelPalaceFrontToBack,
                            HydraPixelPalaceActions.PixelPalaceFrontToBack, true);
                    pixelDropTimer.reset();
                    autonState += 1;
                }
                break;
            case 303:
                if (pixelDropTimer.milliseconds() >= cPixelFrontScoreRunTimeMs) {
                    PixelPalace.Stop();
                    Drive.Start(-6, 0, 0);
                    autonState = 399;
                }
                break;
            case 399:
                if (!Drive.Busy()) {
                    autonState = 400;
                }
                break;
            default:
                // something bad happened
                return false;
        }
        return true;
    }

    /**
     * Drop a pixel at the current location
     * @return false iff something bad happens
     */
    protected boolean PixelDrop(boolean secondPixel) {
        // Drop the pixel on the spike
        switch (autonState) {
            case 100:
                // Reverse the intake
                Intake.StartOut();
                HydraPixelPalaceActions position2Direction = HydraPixelPalaceActions.PixelPalaceStop;
                // if we are dropping the second pixel, we need both servos to turn
                if (secondPixel) {
                    position2Direction = HydraPixelPalaceActions.PixelPalaceBackToFront;
                }
                // Run the pixel out of the cassette
                PixelPalace.Start(HydraPixelPalaceActions.PixelPalaceBackToFront, position2Direction,
                        false);
                // Start a timer since we can't easily detect whether a pixel has exited the intake
                pixelDropTimer.reset();
                autonState += 1;
                break;
            case 101:
                int waitTime = cPixelDropRunTimeMs;
                // Give the second pixel more time to get out
                if (secondPixel) {
                    waitTime *= 2;
                }
                // Wait for our hardcoded timer to elapse
                if (pixelDropTimer.milliseconds() >= waitTime) {
                    // Stop the intake and cassette
                    Intake.Stop();
                    PixelPalace.Stop();
                    // Go to the next major step in the auton
                    autonState = 200;
                }
                break;
            default:
                // Something unexpected happened. Stop everything
                Intake.Stop();
                PixelPalace.Stop();
                return false;
        }
        return true;
    }


    /**
     * Bring the arm home from any position
     * @return false iff something bad happens
     */
    protected boolean ArmToHome() {
        switch (autonState) {
            case 400:
                if (Arm.RunAction(HydraArmMovements.ArmMoveToHome)) {
                    autonState = 500;
                }
                break;
            default:
                // something bad happened
                return false;
        }
        return true;
    }
}
