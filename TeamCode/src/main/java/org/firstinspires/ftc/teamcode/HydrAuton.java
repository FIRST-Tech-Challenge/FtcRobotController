package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "HydrAuton", preselectTeleOp = "HyDrive")
public class HydrAuton extends LinearOpMode {

    private IMU imu;
    HydraArm Arm;
    private HydraDrive Drive;
    HydraPixelPalace PixelPalace;
    private HydraIntake Intake;
    private HydraObjectDetect ObjDet;
    private HydraObjectLocations ObjLoc;
    ElapsedTime pixelDropTimer;
    int autonState;
    int cPixelFrontScoreRunTimeMs;
    int cPixelDropRunTimeMs;
    private long pixelDropTimeStart;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double cWheelDiameter;
        double cWheelCircumference;
        double cCountsPerWheelRevolution;
        double cCountsPerInch;
        double cDriveBoosted;
        double cDriveNormal;
        double cDriveSlow;
        double cCasBackToFront;
        double cCasFrontToBack;
        double cLowerArmAutoMotorPwr;
        double cUpperArmAutoMotorPwr;
        float cXvalueForLeftToCenterObject;
        int cIntakeIn;
        int cIntakeOut;
        int cPixelPos1Dist;
        int cPixelPos2Dist;
        int cMaxObjectSearchTimeMs;
        ElapsedTime opModeTimer;
        boolean autonAbort;

        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize Constant Variables
        // Wheel constants
        cWheelDiameter = 3.78;
        cWheelCircumference = cWheelDiameter * Math.PI;
        cCountsPerWheelRevolution = 537.6;
        cCountsPerInch = cCountsPerWheelRevolution / cWheelCircumference;
        // Drive motor power level scaling [max 1]
        cDriveBoosted = 1;
        cDriveNormal = 0.9;
        cDriveSlow = 0.5;
        // Arm motor power scaling
        cLowerArmAutoMotorPwr = 0.5;
        cUpperArmAutoMotorPwr = 0.4;
        // Servo speeds for the cassette
        cCasFrontToBack = 0.8;
        cCasBackToFront = 0.2;
        // Distance to detect pixels in the cassette (cm)
        cPixelPos1Dist = 1;
        cPixelPos2Dist = 10;
        // Intake motor speeds
        cIntakeIn = -1;
        cIntakeOut = 1;
        // Max x value for an object on the left spike
        cXvalueForLeftToCenterObject = 200;
        // Maximum time to run image recognition to discover the prop
        cMaxObjectSearchTimeMs = 2000;
        // How long to run the servo and intake when dropping the pixel
        cPixelDropRunTimeMs = 2000;
        cPixelFrontScoreRunTimeMs = 2000;
        // Initialize Local Variables
        autonState = 0;
        ObjLoc = HydraObjectLocations.ObjLocUnknown;
        pixelDropTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        autonAbort = false;
        // Initialization Routines
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        Arm = new HydraArm("MotUprArm", "MotLwrArm", cUpperArmAutoMotorPwr,
                cLowerArmAutoMotorPwr);
        Drive = new HydraDrive("MotDrFrLt", "MotDrFrRt", "MotDrBkLt",
                "MotDrBkRt", cCountsPerInch, cDriveBoosted, cDriveNormal, cDriveSlow);
        PixelPalace = new HydraPixelPalace("SrvPxlPos1", "SrvPxlPos2", "LED1",
                "LED2", "LED3", "LED4", "SenColPxlPos1", "SenColPxlPos2",
                cCasFrontToBack, cCasBackToFront, cPixelPos1Dist, cPixelPos2Dist);
        Intake = new HydraIntake("MotPxlIntk", cIntakeIn, cIntakeOut);
        ObjDet = new HydraObjectDetect("Blue_Prop.tflite", cXvalueForLeftToCenterObject);
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
        opModeTimer.reset();
        // Find the object so we can drive to it
        while (opModeIsActive()) {
            ObjLoc = ObjDet.GetObjectLocation();
            if (ObjLoc != HydraObjectLocations.ObjLocUnknown ||
                    opModeTimer.milliseconds() >= cMaxObjectSearchTimeMs) {
                break;
            }
            // Push telemetry to the Driver Station.
            telemetry.update();
            sleep(20);
        }
        // If we did not find it, we have no choice but to assume that it was in the position we can't see
        if (ObjLoc == HydraObjectLocations.ObjLocUnknown) {
            ObjLoc = HydraObjectLocations.ObjLocRightSpike;
        }
        while (opModeIsActive()) {
            if (AutonBlueWing()) {
                break;
            }
            if (opModeTimer.milliseconds() >= 27000) {
                autonAbort = true;
                autonState = 400;
                ArmToHome();
                break;
            }
            telemetry.addData("State", autonState);
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }
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
    }

    /**
     * Describe this function...
     */
    private void BadState() {
        telemetry.addData("InvalidState", autonState);
    }

    /**
     * Describe this function...
     */
    private boolean AutonRedWing() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocLeftSpike) {
                    autonState = 10;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 20;
                } else if (ObjLoc == HydraObjectLocations.ObjLocRightSpike) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(16, -14, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -16, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(30, -3, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocLeftSpike) {
                    autonState = 210;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(34, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    if (!Drive.Busy()) {
                        Drive.Start(76, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 214) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 27, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -19, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(87, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 24, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -22, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    if (!Drive.Busy()) {
                        Drive.Start(76, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 232) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 29, 0);
                        autonState = 300;
                    }
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            if (!Drive.Busy()) {
                ScoreFront();
            }
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            if (!Drive.Busy()) {
                ArmToHome();
            }
        } else if (autonState == 500) {
            if (true) {
                return true;
            }
        } else {
            BadState();
            autonState = 500;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private boolean AutonBlueWing() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocRightSpike) {
                    autonState = 10;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 20;
                } else if (ObjLoc == HydraObjectLocations.ObjLocLeftSpike) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(16, 14, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 15, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(30, 3, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocRightSpike) {
                    autonState = 210;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(30, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    if (!Drive.Busy()) {
                        Drive.Start(73, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 214) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -24, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 19, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(87, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -24, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 22, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    if (!Drive.Busy()) {
                        Drive.Start(76, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 232) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -29, 0);
                        autonState = 300;
                    }
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            if (!Drive.Busy()) {
                ScoreFront();
            }
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            if (!Drive.Busy()) {
                ArmToHome();
            }
        } else if (autonState == 500) {
            if (true) {
                return true;
            }
        } else {
            BadState();
            autonState = 500;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private boolean AutonRedBackstage() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocRightSpike) {
                    autonState = 10;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 20;
                } else if (ObjLoc == HydraObjectLocations.ObjLocLeftSpike) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(16, 14, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 15, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(28, 3, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocRightSpike) {
                    autonState = 210;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(-20, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    autonState += 1;
                } else if (autonState == 214) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -8, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(-17, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(-28, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    autonState += 1;
                } else if (autonState == 232) {
                    // This is the last state for the left spike
                    autonState = 300;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            ScoreBack();
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            ArmToHome();
        } else if (autonState < 600) {
            if (!Drive.Busy()) {
                Drive.Start(0, -36, 0);
                autonState = 600;
            }
        } else if (autonState == 600) {
            if (!Drive.Busy()) {
                return true;
            }
        } else {
            BadState();
            autonState = 600;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private boolean AutonBlueBackstage() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocLeftSpike) {
                    autonState = 10;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 20;
                } else if (ObjLoc == HydraObjectLocations.ObjLocRightSpike) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(13, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -15, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -16, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(30, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (ObjLoc == HydraObjectLocations.ObjLocLeftSpike) {
                    autonState = 210;
                } else if (ObjLoc == HydraObjectLocations.ObjLocCenterSpike) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(-22, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, -12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    autonState += 1;
                } else if (autonState == 214) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 8, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(-17, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(-31, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    autonState += 1;
                } else if (autonState == 232) {
                    // This is the last state for the left spike
                    autonState = 300;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            ScoreBack();
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            ArmToHome();
        } else if (autonState < 600) {
            if (!Drive.Busy()) {
                Drive.Start(0, 32, 0);
                autonState = 600;
            }
        } else if (autonState == 600) {
            if (!Drive.Busy()) {
                return true;
            }
        } else {
            BadState();
            autonState = 600;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private void ScoreBack() {
        if (autonState == 300) {
            if (Arm.RunAction(HydraArmMovements.ArmMoveToBack)) {
                autonState += 1;
            }
        } else if (autonState == 301) {
            if (Arm.RunAction(HydraArmMovements.ArmMoveToBack)) {
                autonState += 1;
            }
        } else if (autonState == 302) {
            if (!Drive.Busy()) {
                Drive.Start(-6, 0, 0);
                autonState += 1;
            }
        } else if (autonState == 303) {
            if (!Drive.Busy()) {
                PixelPalace.Start(HydraPixelPalaceActions.PixelPalaceBackToFront,
                        HydraPixelPalaceActions.PixelPalaceBackToFront, true);
                pixelDropTimer.reset();
                autonState += 1;
            }
        } else if (autonState == 304) {
            if (pixelDropTimer.milliseconds() >= cPixelFrontScoreRunTimeMs) {
                PixelPalace.Stop();
                Drive.Start(4, 0, 0);
                autonState = 400;
            }
        } else {
            BadState();
            autonState = 400;
        }
    }

    /**
     * Describe this function...
     */
    private void ScoreFront() {
        if (autonState == 300) {
            if (Arm.RunAction(HydraArmMovements.ArmMoveToFront)) {
                autonState += 1;
            }
        } else if (autonState == 301) {
            if (Arm.RunAction(HydraArmMovements.ArmMoveToFront)) {
                autonState += 1;
            }
        } else if (autonState == 302) {
            if (!Drive.Busy()) {
                Drive.Start(8, 0, 0);
                autonState += 1;
            }
        } else if (autonState == 303) {
            if (!Drive.Busy()) {
                PixelPalace.Start(HydraPixelPalaceActions.PixelPalaceFrontToBack,
                        HydraPixelPalaceActions.PixelPalaceFrontToBack, true);
                pixelDropTimer.reset();
                autonState += 1;
            }
        } else if (autonState == 304) {
            if (pixelDropTimer.milliseconds() >= cPixelFrontScoreRunTimeMs) {
                PixelPalace.Stop();
                Drive.Start(-6, 0, 0);
                autonState = 400;
            }
        } else {
            BadState();
            autonState = 400;
        }
    }

    /**
     * Describe this function...
     */
    private void PixelDrop() {
        // Drop the pixel on the spike
        if (autonState == 100) {
            // Reverse the intake
            Intake.StartOut();
            // Run one pixel out of the cassette
            PixelPalace.Start(HydraPixelPalaceActions.PixelPalaceBackToFront,
                    HydraPixelPalaceActions.PixelPalaceStop, false);
            // Start a timer since we can't easily detect whether a pixel has exited the intake
            pixelDropTimer.reset();
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            pixelDropTimeStart = System.currentTimeMillis();
            autonState += 1;
        } else if (autonState == 101) {
            // Wait for our hardcoded timer to elapse
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            if (System.currentTimeMillis() - pixelDropTimeStart >= cPixelDropRunTimeMs) {
                // Stop the intake and cassette
                Intake.Stop();
                PixelPalace.Stop();
                // Go to the next major step in the auton
                autonState = 200;
            }
        } else {
            // some bad state. stop everything and move on
            BadState();
            autonState = 200;
            Intake.Stop();
            PixelPalace.Stop();
        }
    }

    /**
     * Describe this function...
     */
    private void ArmToHome() {
        if (autonState == 400) {
            if (Arm.RunAction(HydraArmMovements.ArmMoveToHome)) {
                autonState += 1;
            }
        } else if (autonState == 401) {
            if (Arm.RunAction(HydraArmMovements.ArmMoveToHome)) {
                autonState = 500;
            }
        } else {
            BadState();
            autonState = 500;
        }
    }
}
