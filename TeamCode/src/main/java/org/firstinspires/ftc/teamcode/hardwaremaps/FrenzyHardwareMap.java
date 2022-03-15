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
package org.firstinspires.ftc.teamcode.hardwaremaps;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class FrenzyHardwareMap {
    //Motor in Port 0, Rev Hub 1.
    public DcMotorEx motorFrontLeft = null;
    //Motor in Port 1, Rev Hub 1.
    public DcMotorEx motorFrontRight = null;
    //Motor in Port 2, Rev Hub 1.
    public DcMotorEx motorBackRight = null;
    //Motor in Port 3, Rev Hub 1.
    public DcMotorEx motorBackLeft = null;
    //Motor in Port 0, Rev Hub 2.
    public DcMotor motorIntake = null;
    //Motor in Port 1, Rev Hub 2
    public DcMotorEx motorArm = null;
    //Motor in Port 2, Rev Hub 2
    public DcMotorEx motorCarousel = null;
    //IMU from RevHub.
    public BNO055IMU imu = null;
    //Color Sensor
    public NormalizedColorSensor colorsensed = null;
    //magnetic limit switch for arm
    public TouchSensor armLimitSwitch;
    //Setup Wheel measurements for REV motors.
    //Encoder clicks are originally 28 per rotation, but multiply by 20:1.
    public final int REV_ENCODER_CLICKS = 560;
    final double REV_WHEEL_DIAM = 7.5;
    public final double REV_WHEEL_CIRC = (REV_WHEEL_DIAM/2) * Math.PI;
    final double CLICKS_PER_CM = REV_ENCODER_CLICKS / REV_WHEEL_CIRC;

    //Setup local opmode members.
    HardwareMap frenzyMap = null;
    Telemetry telemetry = null;
    private ElapsedTime period = new ElapsedTime();
    //Constructor
    public FrenzyHardwareMap() {
    }
    //Initialize the hardware interface
    public void init(HardwareMap hwMap, Telemetry frenzyTelemetry) {
        //Save reference to the hardware map.
        frenzyMap = hwMap;
        telemetry = frenzyTelemetry;

        //Define and initialize drivetrain motors.
        motorFrontLeft = frenzyMap.get(DcMotorEx.class, "frontLeft");
        motorBackLeft = frenzyMap.get(DcMotorEx.class, "backLeft");
        motorFrontRight = frenzyMap.get(DcMotorEx.class, "frontRight");
        motorBackRight = frenzyMap.get(DcMotorEx.class, "backRight");

        //Define and initialize arm/intake/carousel motors.
        motorArm = frenzyMap.get(DcMotorEx.class, "arm");
        motorIntake = frenzyMap.get(DcMotorEx.class, "intake");
        motorCarousel = frenzyMap.get(DcMotorEx.class, "carousel");
        //Define the imu
        imu = frenzyMap.get(BNO055IMU.class, "imu");
        //Define Color Sensor
        colorsensed = frenzyMap.get(NormalizedColorSensor.class,"colorsense");

        //define limit switch
        armLimitSwitch = frenzyMap.get(TouchSensor.class,"limitSwitch");

        // Set all motor directions.
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motor directions arm/intake/carousel.
        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
        motorArm.setDirection(DcMotorEx.Direction.REVERSE);
        motorCarousel.setDirection(DcMotorEx.Direction.REVERSE);

        // Set all motors to zero power.
        motorFrontRight.setPower(0.0);
        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);

        // Set arm/intake/carousel motors to zero power.
        motorArm.setPower(0.0);
        motorIntake.setPower(0.0);
        motorCarousel.setPower(0.0);

        // Set all motors to run with encoders.
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set arm motor modes
        motorArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    //Stops and resets the encoders before setting them to run again. drive train specific
    public void restartEncoders(){
            motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    //Sets the powers of all motors.
    public void setPowers(double input){
        motorFrontRight.setPower(input);
        motorFrontLeft.setPower(input);
        motorBackLeft.setPower(input);
        motorBackRight.setPower(input);
    }
    //Checks if all motors are busy
    public boolean motorsBusy(){
        if(motorFrontRight.isBusy() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy()) return true;
        else return false;
    }
    //sets motor target locations to right and left targets
    public void setTargets(int target1, int target2){
        motorFrontLeft.setTargetPosition(target1);
        motorBackLeft.setTargetPosition(target1);
        motorFrontRight.setTargetPosition(target2);
        motorBackRight.setTargetPosition(target2);
    }
    //set motor mode to run To position
    public void setRunToPosition(){
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}

