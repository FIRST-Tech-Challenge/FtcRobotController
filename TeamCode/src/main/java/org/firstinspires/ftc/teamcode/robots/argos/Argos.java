/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.robots.argos;

import android.content.Context;
import android.location.Location;
import android.location.LocationManager;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.robots.UGBot.Autonomous;
import org.firstinspires.ftc.teamcode.robots.UGBot.LEDSystem;
import org.firstinspires.ftc.teamcode.robots.UGBot.PoseUG;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;
import org.firstinspires.ftc.teamcode.util.CsvLogKeeper;
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import org.firstinspires.ftc.teamcode.vision.colorblob.ColorBlobDetector;

import java.util.Locale;

import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.vuforia.HINT;

//import static org.firstinspires.ftc.teamcode.util.VisionUtils.getJewelConfig;
//import static org.firstinspires.ftc.teamcode.util.VisionUtils.getImageFromFrame;


@TeleOp(name="Argos", group="Research")  // @Autonomous(...) is the other common choice
//  @Autonomous

public class Argos extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private PoseArgos robot = new PoseArgos();

    SoundPlayer deadShotSays = SoundPlayer.getInstance();

    private boolean active = true;
    boolean joystickDriveStarted = false;

    private int autoState = 0;
    private int beaconConfig = 0;

    private int flingNumber = 0;
    private boolean shouldLaunch = false;
    private boolean isBlue = false;
    private boolean capMode = false;
    private double pwrDamper = .05;
    private double pwrFwd = 0;
    private double pwrStf = 0;
    private double degreeRot = 0;

    private double vuPwr = 0;

    private ColorBlobDetector mDetector;

    Orientation angles;
    Location location;
    Location home;

    private int state = 0;
    private  int vuTestMode = 0;
    private boolean runAutonomous = true;
    private long autoTimer = 0;
    private long launchTimer = 0;
    private long autoDelay = 0;
    private boolean[] buttonSavedStates = new boolean[11];
    //private boolean[] buttonCurrentState = new boolean[8];
    private boolean slowMode = false;

    private boolean runDemo = false;
    private boolean runBeaconTestLeft = true;
    private int beaconState = 0;



    private int pressedPosition = 750; //Note: find servo position value for pressing position on servoPan
    private int relaxedPosition = 2250; //Note: find servo position value for relaxing position on servoPan

    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    double testableDouble = robot.KpDrive;
    double testableHeading = 0;
    boolean testableDirection = true;

    private int a = 0; //collect (particle mode), manual cap lower (cap mode)
    private int b = 1; //eject (particle mode)
    private int x = 2; //launch (particle mode)
    private int y = 3; //spin up (particle mode), manual cap raise (cap mode)
    private int dpad_down = 4; //no function (particle mode), lowers cap to next lowest preset (cap mode)
    private int dpad_up = 5; //no function (particle mode), raises cap to next highest preset (cap mode)
    private int dpad_left = 6; //toggles between particle and cap mode (both modes)
    private int dpad_right = 7; //no function
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)

    public VuforiaTrackables beaconTargets;
    VuforiaTrackable redNearTarget;
    VuforiaTrackable blueNearTarget;
    VuforiaTrackable redFarTarget;
    VuforiaTrackable blueFarTarget;
    VuforiaLocalizer locale;
    private CsvLogKeeper logger;
    private PoseArgos.RobotType currentBot = PoseArgos.RobotType.Argos;
    private FtcDashboard dashboard;

    // loop time profile
    long lastLoopClockTime;
    double loopAvg = 0;
    private static final double loopWeight = .1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing " + currentBot + "...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();

        robot = new PoseArgos();
        robot.init(this.hardwareMap);

        //auto = new Autonomous(robot, dummyT, gamepad1);

        logger = new CsvLogKeeper("test",3,"tps, armTicks, targetDistance");

/*
        debugTelemetry = gamepad1.right_trigger > .3;
        debugTelemetry = true;
        if (debugTelemetry)
            configureDashboardDebug();
        else
            configureDashboardMatch();

        */
        telemetry.update();

        // waitForStart();
        // this is commented out but left here to document that we are still doing the
        // functions that waitForStart() normally does, but needed to customize it.

        dashboard = FtcDashboard.getInstance();
        robot.resetMotors(true);
        //auto.visionProviderFinalized = false;
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


        stateSwitch();

        if (active) {

            // we can do very basic driving to get to calibration position
            // turret and drive controls on gamepad1 only since we don't always have 2 pads
            // for auton testing

            // this test suppresses pregame driving while a calibration articulation is
            // active
            //if (robot.articulation == PoseUG.Articulation.manual)
            //    joystickDrivePregameMode();

        }

        else { // if inactive we are in configuration mode
/*
            if(auto.visionProviderFinalized)
                auto.sample();

            if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_left, dpad_left, 1)) {
                auto.visionProviderState = (auto.visionProviderState + 1) % auto.visionProviders.length; // switch
                // vision
                // provider
            }
            if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up, 1)) {
                auto.initVisionProvider(); // this is blocking
            } else if (auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up, 1)) {
                auto.deinitVisionProvider(); // also blocking, but should be very quick
            }
            if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_down, dpad_down, 1)) {
                auto.enableTelemetry = !auto.enableTelemetry; // enable/disable FtcDashboard telemetry
                // CenterOfGravityCalculator.drawRobotDiagram =
                // !CenterOfGravityCalculator.drawRobotDiagram;
            }
            if (auto.visionProviderFinalized && gamepad1.left_trigger > 0.3) {
                StackHeight sp = auto.vp.detect();
                if (sp != StackHeight.NONE_FOUND)
                    initStackHeightTest = sp;
                telemetry.addData("Vision", "Prep detection: %s%s", initStackHeightTest,
                        sp == StackHeight.NONE_FOUND ? " (NONE_FOUND)" : "");
            }

            if (soundState == 0 && toggleAllowed(gamepad1.right_stick_button, right_stick_button, 1)) {
                initialization_initSound();
            }

            telemetry.addData("Vision", "Backend: %s (%s)",
                    auto.visionProviders[auto.visionProviderState].getSimpleName(),
                    auto.visionProviderFinalized ? "finalized"
                            : System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : " NOT FINALIZED ");
            telemetry.addData("Vision", "FtcDashboard Telemetry: %s",
                    auto.enableTelemetry ? "Enabled" : "Disabled");
            telemetry.addData("Vision", "Viewpoint: %s", auto.viewpoint);

            */
            telemetry.addData("Status", "Initialized");

        }
        telemetry.update();

        robot.updateSensors(active);


    } // end of stuff that happens during Init, but before Start

    //
    // THIS SECTION EXECUTES ONCE RIGHT AFTER START IS PRESSED
    //
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

/*
        if (auto.vp == null) {
            auto.initDummyVisionProvider(); // this is blocking
        }

        auto.vp.reset();
*/

        lastLoopClockTime = System.nanoTime();
    }

    //
    // END OF SECTION THAT EXECUTES ONCE RIGHT AFTER START IS PRESSED
    //
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        stateSwitch();
        if (active) {
                switch(state){
                    case 0:
                        if(toggleAllowed(gamepad1.b, b)){
                            robot.zeroHead();
                        }
                        if(toggleAllowed(gamepad1.dpad_left, dpad_left)){
                            robot.toggleDriftMode();
                        }
                        if(gamepad1.x){
                            robot.moveArgos((VuforiaTrackableDefaultListener)redNearTarget.getListener(), pwrDamper, 1000);
                        }
                        else {
                            joystickDrive();
                            //robot.vuTargetTracker((VuforiaTrackableDefaultListener)redNearTarget.getListener());
                        }

                        break;
                    case 1: //this is the tertiaryAuto we use if our teamates can also go for the beacons more reliably than we can; scores 2 balls and pushes the cap ball, also parks on the center element
                        joystickDriveStarted = false;

                        break;
                    case 2: //code for tele-op control
                        joystickDrive();
                        break;
                    case 3:
                        robot.PIDTune(robot.balancePID, toggleAllowed(gamepad1.dpad_up,dpad_up), toggleAllowed(gamepad1.dpad_down,dpad_down), toggleAllowed(gamepad1.y,y), toggleAllowed(gamepad1.a,a), toggleAllowed(gamepad1.x,x));
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                case 6:
                    demo();
                    break;
                case 7:
                    break;
                case 8:
                    demo();
                    break;
                case 9:
                    break;
                case 10:

                    break;
                default:
                    robot.stopAll();
                    break;
            }
            robot.updateSensors(active);
        } else {
            //robot.stopAll();
        }

        long loopClockTime = System.nanoTime();
        long loopTime = loopClockTime - lastLoopClockTime;
        if (loopAvg == 0)
            loopAvg = loopTime;
        else
            loopAvg = loopWeight * loopTime + (1 - loopWeight) * loopAvg;
        lastLoopClockTime = loopClockTime;

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void demo(){
        robot.MaintainHeading(gamepad1.x);

    }

    public void joystickDrive(){

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */
        if(toggleAllowed(gamepad1.dpad_left, dpad_left)){
            capMode = !capMode;
        }

        if (!joystickDriveStarted) {
            robot.resetMotors(true);
            joystickDriveStarted = true;
        }
        pwrFwd = pwrDamper * -gamepad1.left_stick_y;
        degreeRot = -gamepad1.right_stick_x * 45; //hard right maps to 45 degree steering

        if(toggleAllowed(gamepad1.y, y)){
            robot.setKdDrive(robot.getKdDrive() + 10);
        }

        if(toggleAllowed(gamepad1.a, a)){
            robot.setKdDrive(robot.getKdDrive() - 10);
        }

        if(toggleAllowed(gamepad1.dpad_up, dpad_up)){
            robot.setKpDrive(robot.getKpDrive() + 0.005);
        }

        if(toggleAllowed(gamepad1.dpad_down, dpad_down)){
            robot.setKpDrive(robot.getKpDrive() - 0.005);
        }

        if (!runDemo && !robot.isBalanceMode())
            robot.driveMixer(pwrFwd, degreeRot);

    }

    public void resetAuto(){
        autoState = 0;
        autoTimer = 0;
    }


    public void stateSwitch(){

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */

        if(toggleAllowed(gamepad1.left_bumper,left_bumper)) {

            state--;
            if (state < 0) {
                state = 9 ;
            }
            robot.resetMotors(true);
            active = false;
            resetAuto();
            beaconState = 0;
        }

        if (toggleAllowed(gamepad1.right_bumper,right_bumper)) {

            state++;
            if (state > 9) {
                state = 0;
            }
            robot.resetMotors(true);
            active = false;
            resetAuto();
            beaconState = 0;
        }

        if(toggleAllowed(gamepad1.start, startBtn)) {
            robot.resetMotors(true);
            active = !active;
            beaconState = 0;
        }

        //auto switch into balance mode when angle climbs over 70 degrees
        if (robot.getRoll()>robot.staticBalance - robot.balanceWindow && robot.getRoll()<robot.staticBalance+robot.balanceWindow) {
            if (!robot.isBalanceMode()) // we are transitioning into balance mode
            {
                robot.nod = .9;
                // this is a tighter requirement, won't enter full balance mode until robot
                // has climbed past balance point
                if (robot.getRoll()>robot.staticBalance){
                    robot.balancePID.setTotalError(0);
                    robot.setBalanceMode(true);
                    robot.incrementNumTimesBalanced();
                }
            }


        }
        else {
            if (robot.isBalanceMode()) // we are transitioning out of balance mode
            {
                robot.nod = .5;
                robot.setHeadTilt(robot.nod);
                robot.motorFront.setPower(0);
            }
            robot.setBalanceMode(false);

        }


    }



    boolean toggleAllowed(boolean button, int buttonIndex)
    {

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */

        if (button) {
            if (!buttonSavedStates[buttonIndex])  { //we just pushed the button, and when we last looked at it, it was not pressed
                buttonSavedStates[buttonIndex] = true;
                return true;
            }
            //       else if(buttonCurrentState[buttonIndex] == buttonSavedStates[buttonIndex] && buttonCurrentState[buttonIndex]){
            else { //the button is pressed, but it was last time too - so ignore

                return false;
            }
        }

        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        return false; //not pressed

    }




    public String getAlliance(){
        if(isBlue)
            return "Blue";
        return "Red";
    }

    public String autoRun(){
        if(runAutonomous)
            return "Auto will run";
        return "Auto will not run";
    }
    public String getTeleopMode(){
        if(capMode){
            return "Cap Mode";
        }
        return "Particle Mode";
    }





    void configureDashboard() {
        // Configure the dashboard.

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                location = robot.getLocation();
            }
        });


        telemetry.addLine()
                .addData("active", new Func<String>() {
                    @Override public String value() {
                        return Boolean.toString(active);
                    }
                })
                .addData("state", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(state);
                    }
                })
                .addData("", new Func<String>() {
                    @Override public String value() {
                        return getTeleopMode();
                    }
                });
//                .addData("Flywheel Speed", new Func<String>() {
//                    @Override public String value() {
//
//                        return Float.toString(robot.particle.flywheelSpeed);
//                    }
//                });
        telemetry.addLine()
//                .addData("Lat", new Func<String>() {
//                    @Override public String value() {
//                        return "" + location. itude();
//                    }
//                })
//                .addData("Long", new Func<String>() {
//                    @Override public String value() {
//                        return "" + location.getLongitude();
//                    }
//                })
//                .addData("Accuracy", new Func<String>() {
//                    @Override public String value() {
//                        return "" + location.getAccuracy();
//                    }
//                })
                .addData("Speed", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.getSpeed();
                    }
                })
                .addData("Bearing", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.getBearing();
                    }
                });

        telemetry.addLine()
                .addData("Kp", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.getKpDrive();
                    }
                })
                .addData("Kd", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.getKdDrive();
                    }
                });

        telemetry.addLine()
                .addData("BKp", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.balancePID.getP();
                    }
                })
                .addData("BKi", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.balancePID.getI();
                    }
                })
                .addData("BKd", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.balancePID.getD();
                    }
                })
                .addData("tuneSt", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.getPidTunerState();
                    }
                })
                .addData("tuneMg", new Func<String>() {
                    @Override public String value() {
                        return "" + robot.getPidTunerMagnitude();
                    }
                });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                })
                .addData("Beacon Config", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(beaconConfig);
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(robot.getHeading());
                    }
                })
                .addData("vuPwr", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(vuPwr);
                    }
                })
                .addData("vuDist", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(robot.getVuDepth());
                    }
                })
                .addData("headingRaw", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);

                    }
                })
                .addData("headingOffset", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.offsetHeading);

                    }
                })

                .addData("rollRaw", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.secondAngle);
                        return Double.toString(robot.getRoll());
                    }
                })
                .addData("pitchRaw", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("State", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(autoState);
                    }
                })
                .addData("TicksFL", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.motorFront.getCurrentPosition());
                    }
                })
                .addData("TicksBL", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.motorBack.getCurrentPosition());
                    }
                })
                .addData("TicksAvg", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.getAverageTicks());
                    }
                });
        telemetry.addLine()

                .addData("PID Calc", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.drivePID.performPID() );
                    }
                })
                .addData("PID Err", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.drivePID.getError());
                    }
                });

        telemetry.addLine()
                .addData("DistRear", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistAft);
                    }
                })

                .addData("DistFore", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistFore);
                    }
                })
                .addData("ForeColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.beaconColor);
                    }
                });

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        //telemetry.addData("Status", "State: " + autoState);
//        //telemetry.addData("Status", "Front Left Ticks: " + Long.toString(motorFront.getCurrentPosition()));
//        //telemetry.addData("Status", "Average Ticks: " + Long.toString(getAverageTicks()));
//        telemetry.addLine().addData("Normal", beaconPresentRear.getLightDetected());
//
//        telemetry.addLine().addData("ColorFore", beaconColorCache[0] & 0xFF);
//        telemetry.addData("ColorRear", ballColorCache[0] & 0xFF);

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }
}
