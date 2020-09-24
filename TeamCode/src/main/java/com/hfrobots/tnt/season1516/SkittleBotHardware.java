/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1516;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public abstract class SkittleBotHardware extends OpMode {

    // Required hardware map
    // Motor Controller 1 (AL00XQ80)
    //     Port 1 - y1MotorDrive (encoder is in port 1)
    //     Port 2 - y2MotorDrive (encoder is in port 2)
    // Motor Controller 2 (AL00XSTZ)
    //     Port 1 - x1MotorDrive (encoder is in port 1)
    //     Port 2 - x2MotorDrive (encoder is in port 2)
    // Motor Controller 3 (AL00UVE0)
    //     Port 1 - winchDrive
    // Core Device Interface Module (mapped as "dim" AL00VCWV)
    //     Port 5 - mr (color sensor)
    //     Port 0 - frontTouchSensor
    // Servo Controller 1
    //     Port 1 - climberDumpServo
    //     Port 2 - winchAimServo

    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * <p/>
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init() {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //
        // The variable below is used to provide telemetry data to a class user.
        //
        warningGenerated = false;
        warningMessage = "Can't map; ";

        //
        // Connect the first color sensor (mapped as "mr")
        //

        /*
        try {
            sensorRGB = hardwareMap.colorSensor.get("mr");

        } catch (Exception exception) {
            appendWarningMessage("sensorRGB");
            Log.e(LOG_TAG, exception.getLocalizedMessage());

            sensorRGB = null;
        } */

        //
        // Connect the touch sensor
        //

        /*
        try {
            frontTouchSensor = hardwareMap.touchSensor.get("frontTouchSensor");
        } catch (Exception exception) {
            appendWarningMessage("frontTouchSensor");
            Log.e(LOG_TAG, exception.getLocalizedMessage());

            frontTouchSensor = null;
        } */

        //
        // Connect the drive wheel motors.
        //

        try {
            y1MotorDrive = hardwareMap.dcMotor.get("y1MotorDrive");
        } catch (Exception p_exeception) {
            appendWarningMessage("y1MotorDrive");
            Log.e(LOG_TAG, p_exeception.getLocalizedMessage());

            y1MotorDrive = null;
        }

        try {
            y2MotorDrive = hardwareMap.dcMotor.get("y2MotorDrive");
        } catch (Exception p_exeception) {
            appendWarningMessage("y2MotorDrive");
            Log.e(LOG_TAG, p_exeception.getLocalizedMessage());

            y2MotorDrive = null;
        }

        try {
            x1MotorDrive = hardwareMap.dcMotor.get("x1MotorDrive");
        } catch (Exception p_exeception) {
            appendWarningMessage("x1MotorDrive");
            Log.e(LOG_TAG, p_exeception.getLocalizedMessage());

            x1MotorDrive = null;
        }

        try {
            x2MotorDrive = hardwareMap.dcMotor.get("x2MotorDrive");
        } catch (Exception p_exeception) {
            appendWarningMessage("x2MotorDrive");
            Log.e(LOG_TAG, p_exeception.getLocalizedMessage());

            x2MotorDrive = null;
        }

        /*
        try {
            climberDumpServo = hardwareMap.servo.get ("climberDumpServo");
        } catch (Exception exception) {
            appendWarningMessage("climberDumpServo");
            Log.e(LOG_TAG, exception.getLocalizedMessage ());

            climberDumpServo = null;
        }
        */

        /*
        try {
            winchDrive = hardwareMap.dcMotor.get("winchDrive");
        } catch (Exception exception) {
            appendWarningMessage("winchDrive");
            Log.e(LOG_TAG, exception.getLocalizedMessage());

            winchDrive = null;
        }
*/
        /*
        try {
            winchAimServo = hardwareMap.servo.get ("winchAimServo");
        } catch (Exception exception) {
            appendWarningMessage("winchAimServo");
            Log.e(LOG_TAG, exception.getLocalizedMessage ());

            winchAimServo = null;
        }
        */
    }

    /**
     * Access whether a warning has been generated.
     */
    boolean wasWarningGenerated() {
        return warningGenerated;
    }

    /**
     * Access the warning message.
     */
    String getWarningMessage()

    {
        return warningMessage;
    }

    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     * <p/>
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void appendWarningMessage(String exceptionMessage) {
        if (warningGenerated) {
            warningMessage += ", ";
        }
        warningGenerated = true;
        warningMessage += exceptionMessage;
    }

    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    protected float scaleMotorPower(float unscaledPower) {

        //
        // Ensure the values are legal.
        //
        float clippedPower = Range.clip(unscaledPower, -1, 1);

        float[] scaleFactors =
                {0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        //
        // Get the corresponding index for the given unscaled power.
        //
        int scaleIndex = (int) (clippedPower * 16.0);

        if (scaleIndex < 0) {
            scaleIndex = -scaleIndex;
        } else if (scaleIndex > 16) {
            scaleIndex = 16;
        }

        final float scaledPower;

        if (clippedPower < 0) {
            scaledPower = -scaleFactors[scaleIndex];
        } else {
            scaledPower = scaleFactors[scaleIndex];
        }

        return scaledPower;
    }

    double getY1MotorDrivePower() {

        if (y1MotorDrive != null) {
            return y1MotorDrive.getPower();
        }

        return 0.0;
    }

    double getY2MotorDrivePower() {

        if (y2MotorDrive != null) {
            return y2MotorDrive.getPower();
        }

        return 0.0;
    }

    double getX1MotorDrivePower() {

        if (x1MotorDrive != null) {
            return x1MotorDrive.getPower();
        }

        return 0.0;
    }

    double getX2MotorDrivePower() {

        if (x2MotorDrive != null) {
            return x2MotorDrive.getPower();
        }

        return 0.0;
    }

    void driveAlongYAxis(double power) {
        setDrivePower(0, 0, power, -power);
    }

    void driveAlong45YAxis(double power) {
        // FIXME: 11/20/15 What goes here?
    }

    void driveAlongXAxis(double power) {
        setDrivePower(power, -power, 0, 0);
    }

    void driveAlong45XAxis(double power) {
        // FIXME: 11/20/15 What goes here?
    }

    protected void setDrivePower(double x1DrivePower, double x2DrivePower,
                       double y1DrivePower, double y2DrivePower) {
        if (y1MotorDrive != null) {
            y1MotorDrive.setPower(y1DrivePower);
        }
        if (y2MotorDrive != null) {
            y2MotorDrive.setPower(y2DrivePower);
        }

        if (x1MotorDrive != null) {
            x1MotorDrive.setPower(x1DrivePower);
        }

        if (x2MotorDrive != null) {
            x2MotorDrive.setPower(x2DrivePower);
        }
    }

    void stopAllDriveMotors() {
        setDrivePower(0, 0, 0, 0);
    }

    public void runUsingEncoders() {
        DcMotor[] motorsWithEncoders = new DcMotor[] { x1MotorDrive, x2MotorDrive, y1MotorDrive, y2MotorDrive };

        for (DcMotor aMotor : motorsWithEncoders) {
            if (aMotor != null) {
                aMotor.setMode
                        (DcMotor.RunMode.RUN_USING_ENCODERS
                        );
            }
        }
    }

    public void runWithoutBraking() {
        DcMotor[] motorsWithEncoders = new DcMotor[] { x1MotorDrive, x2MotorDrive, y1MotorDrive, y2MotorDrive };

        for (DcMotor aMotor : motorsWithEncoders) {
            if (aMotor != null) {
                aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
    }

    public void runWithoutDriveEncoders() {
        DcMotor[] allDriveMotors = new DcMotor[] { x1MotorDrive, x2MotorDrive,
                y1MotorDrive, y2MotorDrive};

        for (DcMotor aMotor : allDriveMotors) {
            if (aMotor != null) {
                aMotor.setMode
                        (DcMotor.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }
    }

    public void resetYDriveEncoder() {
        if (y1MotorDrive != null) {
            y1MotorDrive.setMode
                    (DcMotor.RunMode.RESET_ENCODERS
                    );
        }

    }

    public void resetXDriveEncoder() {
        if (x1MotorDrive != null) {
            x1MotorDrive.setMode
                    (DcMotor.RunMode.RESET_ENCODERS
                    );
        }

    }

    /**
     * Reset both drive wheel encoders.
     */
    public void resetDriveEncoders() {
        //
        // Reset the motor encoders on the drive wheels.
        //
        resetXDriveEncoder();
        resetYDriveEncoder();
    }


    int getYAxisEncoderCount() {
        return getAxisEncoderCount(y1MotorDrive);
    }

    int getXAxisEncoderCount() {
        return getAxisEncoderCount(x1MotorDrive);
    }

    private int getAxisEncoderCount(DcMotor forMotor) {
        int encoderCount = 0;

        if (forMotor != null) {
            encoderCount = forMotor.getCurrentPosition();
        }

        return encoderCount;
    }

    boolean hasYAxisEncoderReached(double count) {
        return Math.abs(getYAxisEncoderCount()) > count;
    }

    boolean hasXAxisEncoderReached(double count) {
        return Math.abs(getXAxisEncoderCount()) > count;
    }

    boolean hasYAxisEncoderReset() {
        return getYAxisEncoderCount() == 0;
    }

    boolean hasXAxisEncoderReset() {
        return getXAxisEncoderCount() == 0;
    }

    private boolean colorSensorLedEnabled = false;

    protected void enableColorSensorLed(boolean on) {
        if (sensorRGB != null) {
            colorSensorLedEnabled = on;
            sensorRGB.enableLed(on);
        }
    }

    protected boolean getColorSensorLedEnabled() {
        return colorSensorLedEnabled;
    }

    protected ColorSensorValues getColorSensorValues() {
        if (sensorRGB != null) {
            return new ColorSensorValues(sensorRGB);
        } else {
            return new ColorSensorValues();
        }
    }

    public static class ColorSensorValues {
        public final int red;
        public final int blue;
        public final int green;
        public final int alpha;

        ColorSensorValues(ColorSensor sensor) {
            red = sensor.red();
            blue = sensor.blue();
            green = sensor.green();
            alpha = sensor.alpha();

        }

        ColorSensorValues() {
            red = 0;
            blue = 0;
            green = 0;
            alpha = 0;
        }

        @Override
        public String toString() {
            return "(RGBA) " + red + ", " + green + ", " + blue + ", " + alpha;
        }
    }

    protected void setWinchAimServoPosition(double pos) {
        // clip to legal values
        double clippedPos = Range.clip
                ( pos
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );

        if (winchAimServo != null) {
            winchAimServo.setPosition (clippedPos);
        }
    }

    protected double getWinchAimServoPosition() {
        if (winchAimServo != null) {
            return winchAimServo.getPosition ();
        }

        return 0.0D;
    }

    protected void setClimberDumpServoPosition(double pos) {
        // clip to legal values
        double clippedPos = Range.clip
                ( pos
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );

        if (climberDumpServo != null) {
           climberDumpServo.setPosition (clippedPos);
        }
    }

    protected double getClimberDumpServoPosition() {
        if (climberDumpServo != null) {
            return climberDumpServo.getPosition ();
        }

        return 0.0D;
    }

    protected boolean isFrontTouchSensorPressed() {
        if (frontTouchSensor == null) {
            return false;
        }

        return frontTouchSensor.isPressed();
    }

    protected void setWinchDrivePower(double power) {
        if (winchDrive != null) {
            winchDrive.setPower(power);
        }
    }

    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean warningGenerated = false;

    /**
     * Store a message to the user if one has been generated.
     */
    private String warningMessage;

    private DcMotor y1MotorDrive;

    private DcMotor y2MotorDrive;

    private DcMotor x1MotorDrive;

    private DcMotor x2MotorDrive;

    protected ColorSensor sensorRGB;

    private Servo climberDumpServo;

    private TouchSensor frontTouchSensor;

    private DcMotor winchDrive;

    private Servo winchAimServo;
}
