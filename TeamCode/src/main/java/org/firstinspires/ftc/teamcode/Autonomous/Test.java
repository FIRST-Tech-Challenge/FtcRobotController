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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.functions.old.lastseson.DataLogger;
import org.firstinspires.ftc.teamcode.functions.Move;
import org.firstinspires.ftc.teamcode.functions.old.lastseson.Pistol;
import org.firstinspires.ftc.teamcode.functions.Rotate;
import org.firstinspires.ftc.teamcode.functions.old.lastseson.Aspirator;
import org.firstinspires.ftc.teamcode.functions.old.toredo.SistemXY;
import org.firstinspires.ftc.teamcode.functions.mv.MVTurnTowardsPoint;
import org.firstinspires.ftc.teamcode.functions.mv.MVVariables;
import org.firstinspires.ftc.teamcode.functions.VoltageReader;
import org.firstinspires.ftc.teamcode.functions.RotationDetector;
import org.firstinspires.ftc.teamcode.functions.old.toredo.PositionCalculator;
import org.firstinspires.ftc.teamcode.functions.AccelerationDetector;

import java.io.IOException;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "AutonomieTest", group = "Concept")

public class Test extends LinearOpMode {

    private boolean ok=false;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public String label=null;
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, right_pistol;
    public ModernRoboticsI2cRangeSensor rangeFinder = null;
    private Servo push_ring;
    private CRServo arm_servo;
    public Move move;
    public Rotate rotate;
    public VoltageReader voltageReader;
    public Aspirator aspirator;
    public Pistol pistol;
    public RotationDetector rotationDetector;
    public MVVariables.MotorHolder motorHolder;
    public PositionCalculator positionCalculator;
    public AccelerationDetector accelerationDetector;
    public MVTurnTowardsPoint MVTurnTowardsPoint;
    public SistemXY sistemXY;
    public DataLogger dataLogger;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        MVTurnTowardsPoint = new MVTurnTowardsPoint();

        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        push_ring = hardwareMap.servo.get("PR");
        //left_pistol = hardwareMap.dcMotor.get("LP");
        right_pistol = hardwareMap.dcMotor.get("RP");
        // arm_servo = hardwareMap.crservo.get("AS");
        // Tell the driver that initialization is complete.
        motorHolder = new MVVariables.MotorHolder(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        telemetry.addData("Status", "Initialized");

        try {
            accelerationDetector = new AccelerationDetector(hardwareMap.get(BNO055IMUImpl.class, "imu"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        move = new Move(motorHolder);
        rotate = new Rotate(motorHolder);
        //  aspirator = new Aspirator(vaccumMotor, spagheteMotor);

        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        pistol = new Pistol(right_pistol, voltageReader);

        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMUImpl.class, "imu"));
        try {
            positionCalculator = new PositionCalculator(move, 115, 30, rotationDetector, voltageReader);
        } catch (IOException e) {
            e.printStackTrace();
        }
        dataLogger = new DataLogger(rotationDetector, VS, pistol, move, positionCalculator, getClass().getName());

        waitForStart();




        while(sistemXY.GoTo(100, 100)){
            TelemetryData();
            sleep(1);
        }


        //sleep(voltageReader.GetWaitTime(360, 3));
        //sleep(20000);
        
        /*
        while(rotationDetector.WaitForRotation(170)){
                telemetry.addData("current rotation: ", rotationDetector.ReturnPositiveRotation());
                telemetry.update();
                rotate.RotateRaw(rotationDetector.Directie(170), rotationDetector.VitezaMotoare(170));
            }
        
        telemetry.addData("final rotation: ", rotationDetector.ReturnPositiveRotation());
        telemetry.addData("starting rotation: ", rotationDetector.ReturnStartingRotation());
      
        */

        telemetry.update();
        rotate.MoveStop();

        sleep(3000);
        telemetry.update();
        //telemetry.addData("Acceleration modul ", accelerationDetector.ReturnDistanta());
        sleep(20000);
    }
    void TelemetryData(){
        dataLogger.writeData(getRuntime());
        telemetry.addData("Current calc position x: ", positionCalculator.ReturnTestX());
        telemetry.addData("Current calc position y: ", positionCalculator.ReturnTestY());
        telemetry.addData("Current calc position z: ", positionCalculator.ReturnTestZ());
        telemetry.addData("Current acc status: ", positionCalculator.ReturnAccStatus());
        telemetry.addData("Current position x: ", positionCalculator.ReturnX());
        telemetry.addData("Current position y: ", positionCalculator.ReturnY());
        telemetry.addData("Current position distance: ", positionCalculator.ReturnDistanta());
        telemetry.addData("Current acc x: ", positionCalculator.ReturnXAcc());
        telemetry.addData("Current acc y: ", positionCalculator.ReturnYAcc());
        telemetry.addData("Current acc z: ", positionCalculator.ReturnZAcc());
        telemetry.addData("Lat x: ", positionCalculator.LaturaX);
        telemetry.addData("Lat y: ", positionCalculator.LaturaY);
        telemetry.update();
    }

}
