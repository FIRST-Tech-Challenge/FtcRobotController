/* Copyright (c) 2017 FIRST. All rights reserved.
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
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Ninjabot
{

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public Servo claw     = null;
    public DcMotor liftArm = null;
    public DcMotor spinner = null;

    BNO055IMU gyro = null;

    static final int REV_ROBOTICS_HDHEX_MOTOR   = 28; // ticks per rotation
    static final int REV_ROBOTICS_HDHEX_20_to_1 = REV_ROBOTICS_HDHEX_MOTOR * 20;

    static final int DRIVE_MOTOR_TICK_COUNTS    = REV_ROBOTICS_HDHEX_20_to_1;
    static final double WHEEL_DIAMETER          = 4.0;

    static final int REV_ROBOTICS_COREHEX_MOTOR   = 4; // ticks per rotation
    static final int REV_ROBOTICS_COREHEX_72_to_1 = REV_ROBOTICS_COREHEX_MOTOR * 72;
    static final int LiftCounts                    = REV_ROBOTICS_COREHEX_72_to_1;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    LinearOpMode control        =  null;
    public Orientation gyroLastAngle = null;
    private ElapsedTime period  = new ElapsedTime();
    static final double     P_DRIVE_COEFF           = 0.0125;

    /* Constructor */
    public Ninjabot(HardwareMap map, LinearOpMode ctrl ){
        init(map, ctrl);
    }

    // save the location of everything to the hardware map
    public void init(HardwareMap ahwMap, LinearOpMode ctrl )
    {
        // Save reference to Hardware map
        hwMap   = ahwMap;
        control = ctrl;

        leftDrive = hwMap.get(DcMotor.class, "RD");
        rightDrive = hwMap.get(DcMotor.class, "LD");
        claw = hwMap.get(Servo.class,"claw");
        liftArm = hwMap.get(DcMotor.class, "arm");
        spinner = hwMap.get(DcMotor.class, "spinner");

        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        gyro = hwMap.get( BNO055IMU.class, "imu");
        gyroLastAngle = new Orientation();
        //gyroGlobalAngle = 0.0;
    }

    // as given
    public void gyroTurn(double speed, double angle){
        while (control.opModeIsActive() && !gyroOnHeading(speed, angle, 0.1)){
            control.telemetry.update();
        }
    }
    //as given
    boolean gyroOnHeading(double speed, double angle, double PCoeff)
    {
        double  error;
        double  steer;
        boolean onTarget = false;
        double  leftSpeed;
        double  rightSpeed;

        error = gyroGetError(angle);

        if (Math.abs(error) <= 1)
        {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = gyroGetSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        driveSetPower( leftSpeed, rightSpeed );

        control.telemetry.addData("Target", "%5.2f", angle);
        control.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        control.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    //as given
    public double gyroGetError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntergratedZValue();
        //while (robotError > 180)  robotError -= 360;
        //while (robotError <= -180) robotError += 360;
        Orientation angles =
                gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - gyroLastAngle.firstAngle;

        if      (deltaAngle < -180)  deltaAngle += 360;
        else if (deltaAngle >  180)  deltaAngle -= 360;

        robotError=0; // this needs to be deleted later
        return robotError;
    }
    //as given
    public double gyroGetSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }
    //as given
    public void driveSetPower( double leftPower, double rightPower )
    {
        rightDrive.setPower(rightPower);
        leftDrive.setPower(leftPower);
    }

}