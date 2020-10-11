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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import org.firstinspires.ftc.teamcode.vision.colorblob.ColorBlobDetector;

import java.util.Locale;

import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.vuforia.HINT;

import static org.firstinspires.ftc.teamcode.util.VisionUtils.getJewelConfig;
import static org.firstinspires.ftc.teamcode.util.VisionUtils.getImageFromFrame;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the tertiaryAuto or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Argos", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//  @Autonomous

public class Argos extends LinearOpMode {

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



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this.hardwareMap, isBlue);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        configureDashboard();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        locale = ClassFactory.createVuforiaLocalizer(params);
        locale.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        Vuforia.setHint (HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        beaconTargets = locale.loadTrackablesFromAsset("FTC_2016-17");
        beaconTargets.get(0).setName("Gears");
        redNearTarget = beaconTargets.get(3);
        redNearTarget.setName("redNear");  // Gears


        beaconTargets.activate();

        mDetector = new ColorBlobDetector();

        // Acquire a reference to the system Location Manager
        LocationManager locationManager = (LocationManager) RC.a().getSystemService(Context.LOCATION_SERVICE);

        //currently set to beach tent outside of hotel on South Padre
        home = new Location("GPS_PROVIDER");
        home.setLatitude(26.13813517);
        home.setLongitude(-97.16818156);


//        waitForStart(); //this is commented out but left here to document that we are still doing the functions that waitForStart() normally does, but needed to customize it.

        while(!isStarted()){    // Wait for the game to start (driver presses PLAY)
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            //beacons.activate();


            stateSwitch();


            if(gamepad1.y){
                flingNumber = 3;
            }
            if(toggleAllowed(gamepad1.x,x)) {

                    isBlue = !isBlue;

            }
            if(toggleAllowed(gamepad1.dpad_down,dpad_down)){

                autoDelay--;
                if(autoDelay < 0) autoDelay = 15;

            }
            if(toggleAllowed(gamepad1.dpad_up, dpad_up)){

                autoDelay++;
                if(autoDelay>15) autoDelay = 0;

            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Number of throws: " + Integer.toString(flingNumber));
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }



        runtime.reset();

        if(!runAutonomous){
            state = 1;
        }



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            stateSwitch();
            if(active) {
                switch(state){
                    case 0: //main tertiaryAuto function that scores 1 or 2 balls and toggles both beacons
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
                            robot.vuTargetTracker((VuforiaTrackableDefaultListener)redNearTarget.getListener());
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
                        vuTest((VuforiaTrackableDefaultListener)redNearTarget.getListener(),500);
                        beaconConfig = VisionUtils.NOT_VISIBLE;
                        beaconConfig = VisionUtils.getJewelConfig(getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener)redNearTarget.getListener(), locale.getCameraCalibration());
                        if (beaconConfig == VisionUtils.BEACON_RED_BLUE) {
                            Log.i("RED", "BLUE");
                        } else if (beaconConfig != VisionUtils.NOT_VISIBLE) {
                            Log.i("BLUE", "RED");
                        } else {
                            Log.i("BEAC", "== -1");
                        }
                        break;
                    case 5: //provides data for forwards/backwards calibration
                        joystickDriveStarted = false;
                        //if(robot.driveForward(false, 1, .5)) active = false;
                        break;
                    case 6: //provides data for left/right calibration
                        joystickDriveStarted = false;
                        if(robot.getAverageAbsTicks() < 2000){
                            robot.driveMixer(0,0);
                        }
                        else robot.driveMixer(0,0);
                        break;
                    case 7: //demo mode
                        demo();
                        break;
                    case 8: //tertiaryAuto demo mode
                        //tertiaryAuto(1.0);
                        //break;
                        testableHeading = robot.getHeading();
                        state++;
                    case 9:

                        break;
                }
                robot.updateSensors();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void vuTest(VuforiaTrackableDefaultListener beaconTarget, double distance){
        if (toggleAllowed(gamepad1.x, x)) {
            vuTestMode++;
            if (vuTestMode > 1) vuTestMode=0;
        }
        switch (vuTestMode) {
            case 0: //do nothing
                break;
            case 1: // drive to center of beacon target
                vuPwr = robot.driveToBeacon(beaconTarget,isBlue, beaconConfig,500, 0.8, false, false);
                break;

        }
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
