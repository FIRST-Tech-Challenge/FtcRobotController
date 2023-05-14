/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drivetests;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.pid.PIDOpenClosed;

@Config
public class RobotHardware {

    //DRIVE MOTORS
    private Motor fL = null;
    private Motor fR = null;
    private Motor rL = null;
    private Motor rR = null;
    private DcMotor m_fL = null;
    private DcMotor m_fR = null;
    private DcMotor m_rL = null;
    private DcMotor m_rR = null;

    //DRIVE VARIABLES
    private boolean fieldCentric = false;
    private double totalSpeed = 0.5;

    //TURNING VARIABLES
    private PIDCoefficientsEx turningCoeffs = null;
    private PIDEx turningPID = null;
    private AngleController turningController = null;
    private PIDOpenClosed turnPID = null;

    //ELEVATOR VARIABLES
    private PIDCoefficientsEx elevatorCoeffs = null;
    private PIDEx elevatorPID = null;
    private double elePos = 0;
    private int elePower = 0;

    //ELEVATOR
    private Motor eleL = null;
    private Motor eleR = null;
    private DcMotor m_eleL = null;
    private DcMotor m_eleR = null;
    private MotorGroup ele = null;

    //MISC
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
    private MecanumDrive drive = null;
    private RevIMU imu = null;

    public RobotHardware(){
    }

    public void init(OpMode opmode)    {

        this.hardwareMap = opmode.hardwareMap;
        this.telemetry = opmode.telemetry;


        //GYRO
        imu = new RevIMU(hardwareMap);
        imu.init();


        //DRIVE
        fL = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        rL = new Motor(hardwareMap, "rL", Motor.GoBILDA.RPM_312);
        rR = new Motor(hardwareMap, "rR", Motor.GoBILDA.RPM_312);

        m_fL = fL.motor;
        m_fR = fR.motor;
        m_rL = rL.motor;
        m_rR = rR.motor;

        m_fL.setDirection(DcMotorSimple.Direction.REVERSE);
        m_rL.setDirection(DcMotorSimple.Direction.REVERSE);

        m_fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(fL, fR, rL, rR);


        //TURNING PID
        turningCoeffs = new PIDCoefficientsEx(2.5, 0.4, .4, 0.25, 2, 0.5);
        turningPID = new PIDEx(turningCoeffs);
        turningController = new AngleController(turningPID);
        turnPID = new PIDOpenClosed(turningController, 0.2);


        //ELEVATOR
        eleL = new Motor(hardwareMap, "eleL", Motor.GoBILDA.RPM_312);
        eleR = new Motor(hardwareMap, "eleR", Motor.GoBILDA.RPM_312);

        m_eleL = eleL.motor;
        m_eleR = eleR.motor;

        m_eleR.setDirection(DcMotorSimple.Direction.REVERSE);

        m_eleL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_eleR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_eleL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_eleR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_eleL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_eleR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ele = new MotorGroup(eleL, eleR);
        ele.setRunMode(Motor.RunMode.RawPower);



        //ELEVATOR PID
        elevatorCoeffs = new PIDCoefficientsEx(1, 0, 0, 0.25, 2, 0.5);
        elevatorPID = new PIDEx(elevatorCoeffs);

    }

    public void setElevatorPosition(double target) {
       elePower = (int) elevatorPID.calculate(target, elePos);
       ele.set(elePower);
    }

    public void updateElevatorPosition() {
        double elePos1 = m_eleL.getCurrentPosition();
        double elePos2 = m_eleR.getCurrentPosition();
        elePos = (elePos1 + elePos2) / 2;
    }

    public int getElePower() {
        return elePower;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void setFieldCentric(boolean enabled) {
        fieldCentric = enabled;
    }

    public double getHeading() {
        return imu.getRotation2d().getDegrees();
    }

    public void debug() {
        telemetry.addLine();
    }


    public void robotGoSkrtSkrt( double strafeSpeed, double forwardSpeed, double turnSpeed) {
        strafeSpeed *= totalSpeed;
        forwardSpeed *= totalSpeed;
        turnSpeed *= totalSpeed;
        if (fieldCentric) {
            drive.driveFieldCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    imu.getRotation2d().getDegrees()
            );
        } else {
            drive.driveRobotCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed
            );
        }
    }

    public double getTurnAmount(double stick) {
        return turnPID.calculate(stick, Math.toRadians(getHeading()));
    }

    public double getAngleTarget() {
        return turnPID.getTarget();
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    public double isInComingToRest() {
        if (turnPID.isComingToRest()) {
            return 10;
        } else {
            return 0;
        }
    }

    public void setTotalSpeed(double speed) {
        if (speed > 0 && speed <= 1) {
            totalSpeed = speed;
        } else if (speed > 1) {
            totalSpeed = 1;
        }
    }

}
