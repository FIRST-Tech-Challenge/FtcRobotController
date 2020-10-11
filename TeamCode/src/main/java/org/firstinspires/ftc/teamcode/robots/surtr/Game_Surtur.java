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
package org.firstinspires.ftc.teamcode.robots.surtr;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.vision.colorblob.ColorBlobDetector;

import java.util.Locale;

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
@Disabled
@TeleOp(name="Game_Surtr", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//  @Autonomous

public class Game_Surtur extends LinearOpMode {

    //hi im testing something

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private PoseSurtr Surtr = new PoseSurtr();


    private boolean active = true;
    boolean joystickDriveStarted = false;
    public boolean suppressJoysticks = false;
    boolean balancing = false;

    private int state = 0;
    private boolean isBlue = false;


    //drive train control variables
    private double pwrDamper = 1;
    private double pwrFwd = 0;
    private double pwrStf = 0;
    private double pwrRot = 0;
    private double pwrFwdL = 0;
    private double pwrStfL = 0;
    private double pwrFwdR = 0;
    private double pwrStfR = 0;
    private boolean enableTank = false;
    private int direction = -1;  //-1 to reverse direction


    //staging and timer variables
    private int autoStage = 0;
    private int autoSetupStage = 0;
    private  int vuTestMode = 0;


    //vision objects/vision-based variables
    public VuforiaTrackables relicCodex;
    public int savedVuMarkCodex = 0;
    VuforiaTrackable relicTemplate;
    VuforiaLocalizer locale;
    private ColorBlobDetector mDetector;
    private int beaconConfig = 0;
    private double vuPwr = 0;
    public boolean vuFlashDemo = false;
    boolean visionConfigured = false;


    //sensors/sensing-related variables
    Orientation angles;
    boolean vuActive = false;


    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    double testableDouble = Surtr.kpDrive;
    double testableHeading = 0;
    boolean testableDirection = true;


    //values associated with the buttons in the toggleAllowed method
    private boolean[] buttonSavedStates = new boolean[11];
    private int a = 0; //lower glyph lift
    private int b = 1; //toggle grip/release on glyph
    private int x = 2; //no function
    private int y = 3; //raise glyph lift
    private int dpad_down = 4; //glyph lift bottom position
    private int dpad_up = 5; //glyph lift top position
    private int dpad_left = 6; //no function
    private int dpad_right = 7; //glyph lift mid position
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)




    @Override
    public void runOpMode() throws InterruptedException {

        Surtr.init(this.hardwareMap, isBlue);

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

        //set vuforia to look for assets from the Relic Recovery library
        relicCodex = locale.loadTrackablesFromAsset("RelicVuMark");
        relicCodex.get(0).setName("RelicTemplate");


        relicTemplate = relicCodex.get(0);

//        waitForStart(); //this is commented out but left here to document that we are still doing the functions that waitForStart() normally does, but needed to customize it.

        //activate vuforia to start identifying targets/vuMarks
//        relicCodex.activate();
        Surtr.resetMotors(true);

        mDetector = new ColorBlobDetector();

        while(!isStarted()){    // Wait for the game to start (driver presses PLAY)



            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }


            stateSwitch();


            if(toggleAllowed(gamepad1.b, b)){
                relicCodex.activate();
                vuActive = true;

            }
//            else if(toggleAllowed(gamepad1.start, startBtn) && state==0){
//                relicCodex.deactivate();
//            }
            if(toggleAllowed(gamepad1.x,x)) {

                    isBlue = !isBlue;

            }
            if(vuActive){
                telemetry.addData("Vu", "Active");
            }
            else{
                telemetry.addData("Vu", "Inactive");
            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }




//        Surtr.jewel.liftArm();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            stateSwitch();
            if(active) {
                switch(state){
                    case 0: //code for tele-op control
                        joystickDrive();
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        demo((VuforiaTrackableDefaultListener) relicTemplate.getListener(),500);
                        break;
                    case 5: //provides data for forwards/backwards calibration
                        joystickDriveStarted = false;
                        if(Surtr.driveForward(true, .4, .35)) {
                            state = 0;
                            active = false;
                        }
                        break;
                    case 6: //provides data for left/right calibration
                        joystickDriveStarted = false;
                        if(Surtr.driveIMUDistance(Surtr.kpDrive, .5, 0, false, .5, false)) active = false;
                        break;
                    case 7:
                        break;
                    case 8: //servo testing mode
//                        Surtr.servoTester(toggleAllowed(gamepad1.dpad_up, dpad_up), toggleAllowed(gamepad1.y, y), toggleAllowed(gamepad1.a,a), toggleAllowed(gamepad1.dpad_down, dpad_down));
//                        Surtr.relicArm.closeGrip();
                        break;
                    case 9:
                        break;
                    case 10:
                        break;
                    default:
                        Surtr.stopAll();
                        break;
                }
                Surtr.updateSensors();
            }
            else {
                Surtr.stopAll();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }




    public void demo(VuforiaTrackableDefaultListener beaconTarget, double distance){
        if(gamepad1.x){
            Surtr.maintainHeading(gamepad1.x);
        }
        else {


        }
        if(gamepad1.y) {
            Surtr.driveToBeacon(beaconTarget, isBlue, 0, distance, .5, true, false);
        }


        if(gamepad1.a) {
            Surtr.driveToBeacon(beaconTarget, isBlue, 0, distance, .5, false, false);
        }

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

        if (!joystickDriveStarted) {
            Surtr.resetMotors(true);
            joystickDriveStarted = true;
        }

//        else{
//            if(toggleAllowed(gamepad1.b, b)){
//                Surtr.glyphSystem.togglePhoneTilt();
//            }
//        }

        if (balancing) { //balance with a simple drive forward from edge of stone

            if(Surtr.driveForward(true, .42, .45)){
                suppressJoysticks=false;
                Surtr.resetMotors(true);
                balancing = false;
                }
        }
            else {
                suppressJoysticks = false;

            }

//        if(relicMode){
//            Surtr.glyphSystem.tiltPhoneMax();
//        }




        pwrFwd = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStf = direction * pwrDamper * gamepad1.left_stick_x;
        pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;


//        pwrRot += .33 * (gamepad1.right_trigger - gamepad1.left_trigger);

        pwrFwdL = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStfL = direction * pwrDamper * gamepad1.left_stick_x;

        pwrFwdR = direction * pwrDamper * gamepad1.right_stick_y;
        pwrStfR = direction * pwrDamper * gamepad1.right_stick_x;

        if (!suppressJoysticks) {
            if (enableTank) {
//            Surtr.driveMixerMecTank(pwrFwdL, pwrStfL, pwrFwdR, pwrStfR);
                Surtr.driveMixerMecField(pwrFwd, pwrStf, pwrRot, Surtr.getHeading());
            } else {
                Surtr.driveMixerMec(pwrFwd, pwrStf, pwrRot);
            }
        }

        if (toggleAllowed(gamepad1.right_bumper, right_bumper)) {
            if (pwrDamper != .33) {
                pwrDamper = .33;
            } else
                pwrDamper = 1.0;
        }

        if(toggleAllowed(gamepad1.x, x)){
            Surtr.toggleCannon();
        }
    }


    //the method that controls the main state of the Surtr; must be called in the main loop outside of the main switch
    public void stateSwitch() {

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

        if(!active) {
            if (toggleAllowed(gamepad1.left_bumper, left_bumper)) {

                state--;
                if (state < 0) {
                    state = 10;
                }
                Surtr.resetMotors(true);
                active = false;
                }

            if (toggleAllowed(gamepad1.right_bumper, right_bumper)) {

                state++;
                if (state > 10) {
                    state = 0;
                }
                Surtr.resetMotors(true);
                active = false;
            }

//            if (!active) {
//                if (toggleAllowed(gamepad1.b, b)) {
//
//                    state++;
//                    if (state > 10) {
//                        state = 0;
//                    }
//                    Surtr.resetMotors(true);
//                    active = false;
//                    resetAuto();
//                    codexFlashStage = 0;
//
//                }
//            }
        }

        if (toggleAllowed(gamepad1.start, startBtn)) {
            Surtr.resetMotors(true);
            active = !active;
        }
    }


    //checks to see if a specific button should allow a toggle at any given time; needs a rework
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
                angles = Surtr.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
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
                });
//                .addData("servoJewelExtender", new Func<String>() {
//                    @Override public String value() {
//                        return Integer.toString(Surtr.jewel.jewelPos);
//                    }
//                });
//        telemetry.addLine()
//                .addData("Kp", new Func<String>() {
//                    @Override public String value() {
//                        return "" + Surtr.getKpDrive();
//                    }
//                })
//                .addData("Kd", new Func<String>() {
//                    @Override public String value() {
//                        return "" + Surtr.getKdDrive();
//                    }
//                });
//
        telemetry.addLine()
//                .addData("phone pos", new Func<String>() {
//                    @Override public String value() {
//                        return Integer.toString(Surtr.glyphSystem.maintainPhoneTilt());
//                    }
//                })
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return Surtr.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return Surtr.imu.getCalibrationStatus().toString();
                    }
                });
//                .addData("Jewel Red", new Func<String>() {
//                    @Override public String value() {
//                        return "" + Surtr.colorJewel.red();
//                    }
//                })
//
//                .addData("Jewel Blue", new Func<String>() {
//                    @Override public String value() {
//                        return "" + Surtr.colorJewel.blue();
//                    }
//                })

//                .addData("Relic Codex", new Func<String>() {
//                    @Override public String value() {
//                        return getRelicCodexStr();
//                    }
//                })
//                .addData("Relic Codex", new Func<String>() {
//                    @Override public String value() {
//                        return Integer.toString(savedVuMarkCodex);
//                    }
//                });

//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(Surtr.getHeading());
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(Surtr.getPitch());
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(Surtr.getRoll());
//                    }
//                });
//                .addData("glyph roll", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Double.toString(Surtr.glyphSystem.roll);
//                    }
//                })
//                .addData("glyph ticks", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.firstAngle);
//                        return Integer.toString(Surtr.glyphSystem.getMotorLiftPosition());
//                    }
//                });
//                .addData("headingRaw", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//
//                    }
//                })
//                .addData("headingOffset", new Func<String>() {
//                    @Override public String value() {
//                        return Double.toString(Surtr.offsetHeading);
//
//                    }
//                })
//
//                .addData("rollRaw", new Func<String>() {
//                    @Override public String value() {
//                        //return formatAngle(angles.angleUnit, angles.secondAngle);
//                        return Double.toString(Surtr.getRoll());
//                    }
//                })
//                .addData("pitchRaw", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//        telemetry.addLine()
//                .addData("auto stage", new Func<String>() {
//                    @Override public String value() {
//                        return String.valueOf(autoStage);
//                    }
//                })
                //.addData("glyph distance", new Func<String>() {
                //    @Override public String value() {
                //        return String.valueOf(Surtr.glyphUpper.getDistance(DistanceUnit.CM));
                //    }
                //})
//                .addData("TicksFL", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(Surtr.motorFront.getCurrentPosition());
//                    }
//                })
//                .addData("TicksBL", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(Surtr.motorBack.getCurrentPosition());
//                    }
//                })
//                .addData("TicksAvg", new Func<String>() {
//                    @Override public String value() {
//                        return Long.toString(Surtr.getAverageTicks());
//                    }
//                });
//        telemetry.addLine()
//
//                .addData("PID Calc", new Func<String>() {
//                    @Override public String value() {
//                        return Double.toString(Surtr.drivePID.performPID() );
//                    }
//                })
//                .addData("PID Err", new Func<String>() {
//                    @Override public String value() {
//                        return Double.toString(Surtr.drivePID.getError());
//                    }
//                });
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
