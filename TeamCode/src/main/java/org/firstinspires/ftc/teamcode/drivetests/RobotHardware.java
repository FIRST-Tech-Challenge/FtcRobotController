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
import org.firstinspires.ftc.teamcode.classes.PIDOpenClosed;

@Config
public class RobotHardware {

    //DRIVE
    private Motor fL = null;
    private Motor fR = null;
    private Motor rL = null;
    private Motor rR = null;
    private DcMotor m_fL = null;
    private DcMotor m_fR = null;
    private DcMotor m_rL = null;
    private DcMotor m_rR = null;

    private boolean fieldCentric = false;
    private double totalSpeed = 0.5;
    private double deadzone = 0.1;
    private PIDCoefficientsEx turningCoeffs = null;
    private PIDEx turningPID = null;
    private AngleController turningController = null;
    private PIDOpenClosed turnPID = null;

    public static double angleTarget = 0;
    public static boolean turnLeftTog = false;
    public static boolean turnRightTog = false;
    public static double veloDeadzone = 5;
    public static double finalStick = 0;
    public static double pastHeading = 0;

    //ELEVATOR
    private Motor eleL = null;
    private Motor eleR = null;
    private DcMotor m_eleL = null;
    private DcMotor m_eleR = null;

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

        turningCoeffs = new PIDCoefficientsEx(1, 0, 0, 0.5, 0.5, 0.5);
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

        MotorGroup ele = new MotorGroup(eleL, eleR);

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

//    public double getTurnAmount(double stick) {
//        if (Math.abs(stick) > deadzone) {
//            finalStick = stick;
//            pastHeading = getHeading();
//            angleTarget = getHeading();
//            if (stick < 0) {
//                turnLeftTog = true;
//            } else if (stick > 0) {
//                turnRightTog = true;
//            }
//        } else if (turnLeftTog) {
//            if (pastHeading > getHeading()) {
//                angleTarget = getHeading();
//                turnLeftTog = false;
//            }
//            pastHeading = getHeading();
//            finalStick = -turningController.calculate(Math.toRadians(angleTarget), Math.toRadians(getHeading()));
//        } else if (turnRightTog) {
//            if (pastHeading < getHeading()) {
//                angleTarget = getHeading();
//                turnRightTog = false;
//                }
//            pastHeading = getHeading();
//            finalStick = -turningController.calculate(Math.toRadians(angleTarget), Math.toRadians(getHeading()));
//        } else {
//            finalStick = -turningController.calculate(Math.toRadians(angleTarget), Math.toRadians(getHeading()));
//        }
//        return finalStick;
//    }

    public void turningPIDSetter(double Kp, double Ki, double Kd, double maximumIntegralSum, double stabilityThreshold, double lowPassGain) {
        turningCoeffs = new PIDCoefficientsEx(Kp, Ki, Kd, maximumIntegralSum, stabilityThreshold, lowPassGain);
        turningPID = new PIDEx(turningCoeffs);
        turningController = new AngleController(turningPID);
    }



    public double getAngleTarget() {
        return turnPID.getTarget();
    }

    public void setAngleTarget(double angle) {
        angleTarget = angle;
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    public boolean isTurnLeftTog() {
        return turnLeftTog;
    }

    public boolean isTurnRightTog() {
        return turnRightTog;
    }

    public void setTotalSpeed(double speed) {
        if (speed > 0 && speed <= 1) {
            totalSpeed = speed;
        } else if (speed > 1) {
            totalSpeed = 1;
        }
    }

}
