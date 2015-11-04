/* Copyright (c) 2015 Craig MacFarlane

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Craig MacFarlane nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.MatrixDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashSet;
import java.util.Set;

/**
 * A simple example of all motors and servos oscillating
 */
public class MatrixControllerDemo extends OpMode {

    private ElapsedTime motorOscTimer = new ElapsedTime(0);
    private ElapsedTime servoOscTimer = new ElapsedTime(0);
    private ElapsedTime spamPrevention = new ElapsedTime(0);

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private Set<DcMotor> motorSet = new HashSet<DcMotor>();

    private Servo servo1;
    private Servo servo2;
    private Servo servo3;
    private Servo servo4;

    private MatrixDcMotorController mc;
    private ServoController sc;

    private boolean loopOnce = false;
    private boolean firstMotors = true;
    private boolean firstServos = true;
    private boolean firstBattery = true;
    private int battery;

    private final static double MOTOR_OSC_FREQ = 2.0;
    private final static double SERVO_OSC_FREQ = 1.0;
    private final static double SPAM_PREVENTION_FREQ = 1.0;

    private double motorPower = 1.0;
    private double servoPosition = 0.0;

    @Override
    public void init()
    {
        motor1 = hardwareMap.dcMotor.get("motor_1");
        motor2 = hardwareMap.dcMotor.get("motor_2");
        motor3 = hardwareMap.dcMotor.get("motor_3");
        motor4 = hardwareMap.dcMotor.get("motor_4");

        /*
         * A set of motors to use with the Matrix motor controller's
         * pending feature.  See example below.  Note that this is
         * completely optional.
         */
        motorSet.add(motor1);
        motorSet.add(motor2);
        motorSet.add(motor3);
        motorSet.add(motor4);

        servo1 = hardwareMap.servo.get("servo_1");
        servo2 = hardwareMap.servo.get("servo_2");
        servo3 = hardwareMap.servo.get("servo_3");
        servo4 = hardwareMap.servo.get("servo_4");

        /*
         * Matrix controllers are special.
         *
         * A Matrix controller is one controller with both motors and servos
         * but software wants to treat it as two distinct controllers, one
         * DcMotorController, and one ServoController.
         *
         * We accomplish this by appending Motor and Servo to the name
         * given in the configuration.  In the example below the name
         * of the controller is "MatrixController" so the motor controller
         * instance is "MatrixControllerMotor" and the servo controller
         * instance is "MatrixControllerServo".
         */
        mc = (MatrixDcMotorController)hardwareMap.dcMotorController.get("MatrixController");
        motor1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motor2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motor3.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motor4.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        /*
         * Servos are not enabled by default.  Software must call pwmEnable()
         * for servos to function.
         */
        sc = hardwareMap.servoController.get("MatrixController");
        sc.pwmEnable();
    }

    @Override
    public void start()
    {
        motorOscTimer.reset();
        servoOscTimer.reset();
        spamPrevention.reset();
    }

    @Override
    public void stop()
    {
        /*
         * An example of setting power for individual motors as normal.
         *
         * For the Matrix controller, the methods take effect immediately
         * as each call to setPower(), or any other method that interacts
         * with the controller, is transformed into an i2c transaction and
         * queued.  A separate thread is processing the queue.
         *
         * In practice this means that the first call to setPower will
         * be applied 20 to 40 milliseconds before the last call as the
         * processing thread works through the queue.  Testing
         * has shown that this latency is not large enough to have any
         * real world negative impacts, however teams may choose to use
         * the controller's setMotorPower() method if they desire precise
         * simultaneous motor operations.  See example in handleMotors().
         */
        motor1.setPower(0.0);
        motor2.setPower(0.0);
        motor3.setPower(0.0);
        motor4.setPower(0.0);
        sc.pwmDisable();
    }

    /*
     * handleMotors
     *
     * Oscillate the motors.
     */
    protected void handleMotors()
    {
        if ((firstMotors) || (motorOscTimer.time() > MOTOR_OSC_FREQ)) {
            motorPower = -motorPower;

            /*
             * The MatrixDcMotorController's setMotorPower() method may take
             * a collection of motors.  If this is chosen, then the controller will
             * set a pending bit.  The pending bit tells the controller to
             * defer turning on, or changing the current set point, for a motor
             * until the pending bit is cleared.
             *
             * When the pending bit is cleared all motor power values are applied
             * simultaneously.  setMotorPower() handles the pending bit for you.
             */
            mc.setMotorPower(motorSet, motorPower);
            motorOscTimer.reset();
            firstMotors = false;
        }
    }

    /*
     * handleServos
     *
     * Oscillate the servos.
     */
    protected void handleServos()
    {
        if ((firstServos) || (servoOscTimer.time() > SERVO_OSC_FREQ)) {
            if (servoPosition == 0.0) {
                servoPosition = 1.0;
            } else {
                servoPosition = 0.0;
            }
            servo1.setPosition(servoPosition);
            servo2.setPosition(servoPosition);
            servo3.setPosition(servoPosition);
            servo4.setPosition(servoPosition);
            servoOscTimer.reset();
            firstServos = false;
        }
    }

    /*
     * handleBattery
     *
     * The Matrix controller has a separate battery whose voltage can be read.
     */
    protected void handleBattery()
    {
        if ((firstBattery) || (spamPrevention.time() > SPAM_PREVENTION_FREQ)) {
            battery = mc.getBattery();
            spamPrevention.reset();
            firstBattery = false;
        }
        telemetry.addData("Battery: ", ((float)battery/1000));
    }

    @Override
    public void loop()
    {
        handleMotors();
        handleServos();
        handleBattery();
    }
}
