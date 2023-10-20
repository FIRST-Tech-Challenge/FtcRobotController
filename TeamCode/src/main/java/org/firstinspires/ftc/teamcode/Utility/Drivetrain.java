package org.firstinspires.ftc.teamcode.Utility;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot.AutoDrive.ToTeamProp;

import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Copyright (c) 2019 ParkCircus Productions. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Modified by
 * Matha Goram
 * Hardware map
 * Device name      Control Hub setting
 * imu              I2C bus 0 port 0
 * motorLeftFront   port 0
 * motorLeftBack    port 1
 * motorRightFront  port 2
 * motorRightBack   port 3
 * sensorTouch      n/a
 * sensorLED        n/a
 * sensorColor
 * gamepad1         USB2
 * gamepad2         USB2
 */
public class Drivetrain {

    ToTeamProp ttp_;
    IMU imu_;
    private DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    private final String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.6898396; // 5203-2402-0019  goBILDA 5203 series 19.2:1 ratio
    static final double DRIVE_GEAR_REDUCTION = 1.0;         // already applied to COUNTS_PER_MOTOR_REV
    static final double WHEEL_DIAMETER_INCHES = 3.779528;    // 96 mm Mecanum wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;

    double headingLast = 0.0, headingGlobal = 0.0;
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;

    /**
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     */
    private void resetAngle()
    {
        /*
         * Returns the absolute orientation of the sensor as
         * a set three angles with indicated parameters
         * reference - the axes reference in which the result will be expressed
         * order - the axes order in which the result will be expressed
         * angleUnit - the angle units in which the result will be expressed
         * lastAngles - absolute orientation of the sensor
         */
        headingLast = orientation.getYaw(AngleUnit.DEGREES);
        headingGlobal = 0.0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        orientation = imu_.getRobotYawPitchRollAngles();
        double headingCurrent = orientation.getYaw(AngleUnit.DEGREES);
        double deltaAngle = headingCurrent - headingLast;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        headingLast = headingCurrent;
        headingGlobal += deltaAngle;

        return headingGlobal;
    }

    /**
     * If the current cumulative heading angle is not zero then the robot is
     * not travel in a straight line. Use a simple proportional control to
     * correct the deviation.
     * <p>
     *     The gain value is essentially the proportional control correction factor.
     *     This value may require some tuning based on field experiments.
     * </p>
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        double correction, angle, gain = 0.0055;

        angle = getAngle();
        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Function to stop power to all defined motors in the arraylist motor[]
     * and to wait for 1 second.
     * The stopping action is controlled by the setting chosen for the motor
     * based on vendor specification by the program during the initialization.
     */
    private void fullStop() {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);      // since the following RunMode varies with motors play it safe with this call
            dcMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // see note, if any, on vendor specs for the corresponding motor
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        sleep(250);                     // allow the motors to come to a full stop
    }



    public void Forward(double distance) {
        // Retrieve Rotational Angles and Velocities
        orientation = imu_.getRobotYawPitchRollAngles();
        angularVelocity = imu_.getRobotAngularVelocity(AngleUnit.DEGREES);
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * <p>
     * getAngle() returns + when rotating counter clockwise (left) and - when rotating
     * clockwise (right).
     * </p>
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        resetAngle();                   // reset heading for tracking with IMU data

        if (degrees < 0)
        {                               // turn right
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {                               // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        motor[0].setPower(leftPower);   // set power to rotate
        motor[1].setPower(rightPower);
        motor[2].setPower(rightPower);
        motor[3].setPower(leftPower);
        if (degrees < 0)                // rotate until turn is completed
        {
            // On right turn we have to get off zero first.
            //noinspection StatementWithEmptyBody
            while (ttp_.opModeIsActive() && getAngle() == 0) {}

            //noinspection StatementWithEmptyBody
            while (ttp_.opModeIsActive() && getAngle() > degrees) {}
        }
        else                                // left turn.
            //noinspection StatementWithEmptyBody
            while (ttp_.opModeIsActive() && getAngle() < degrees) {}

        fullStop();

        resetAngle();                          // reset angle tracking on new heading
    }

    public Drivetrain(ToTeamProp ttp, IMU imu) {

        ttp_ = ttp;
        new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception
        imu_ = imu;
        imu_.initialize(new IMU.Parameters(orientationOnRobot));


    // The strings used here must correspond
    // to the names assigned during the robot configuration step on the DS or RC devices.
        for (int i = 0; i < motor.length; i++) {
            // https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/HardwareMap.DeviceMapping.html
            motor[i] = (DcMotorEx) ttp.hardwareMap.get(motorLabels[i]);
            // motor stops and then brakes actively resisting any external force which attempts to turn the motor
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // your settings may vary
        motor[0].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftFront
        motor[1].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.FORWARD); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.FORWARD); // motorRightBack

    }
}