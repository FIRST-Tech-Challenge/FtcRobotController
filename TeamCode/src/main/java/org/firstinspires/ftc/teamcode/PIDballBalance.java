/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.variable.ConfigVariable;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.drive.opmode.servoMovement;

/**
 * {@link PIDballBalance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
//defining our op mod
@Config
@TeleOp(name = "PID ball balance", group = "Sensor")
//defining class "PIDballBalance"
public class PIDballBalance extends LinearOpMode {

    //hardware
    private DistanceSensor sensorRange;
    Servo balServo;

    //defining all of our variables
    public static double kp = 0.05;
    public static double ki = 0.01;
    public static double kd = 60;

    public static double PID_p_error = 0.0;
    public static double PID_i_error = 0.0;
    public static double PID_d_error = 0.0;
    public static double PID_total = 0.0;

    public static double distance = 0.0;
    public static double error = 0.0;
    public static double previous_error = 0.0;
    public static double setValue = 5;

    boolean running = false;
    double servoPosition = 0.0;

    // runopmode method
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //telemetry.addLine("Ready!");
        //telemetry.update();
        //telemetry.clearAll();
        //hardware mapping our hardware
        sensorRange = hardwareMap.get(DistanceSensor.class, "distSensor");
        balServo = hardwareMap.get(Servo.class, "balServo");

        ElapsedTime timer_1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        timer_1.startTime();
        balServo.setPosition(0.00);



        //waiting for start
        waitForStart();
        if (isStopRequested()) return;
        /*
        while (!isStopRequested()) {
            telemetry.addData("mode", Mode.TUNING_MODE);
        }*/
        while (opModeIsActive()) {

            distance = sensorRange.getDistance(DistanceUnit.CM);
            error = distance - setValue;

            if(running) {
                PID_p_error = kp * error;
                PID_d_error = kd * (error - previous_error) / timer_1.milliseconds();

                if (-2 < error && error < 2) {
                    PID_i_error = 0;
                } else {
                    PID_i_error = ki * error + PID_i_error;
                }
            }

            PID_total = PID_p_error + PID_i_error + PID_d_error;
            servoPosition = myMap(PID_total,-17.0, 10.0, 0.62, 0.0);

            if(gamepad1.dpad_up)            ki+=0.005;
            else if(gamepad1.dpad_down)     ki-=0.005;

            if(gamepad1.dpad_right)            setValue+=0.5;
            else if(gamepad1.dpad_left)     setValue-=0.5;

            //if(gamepad2.dpad_up)
                //ki+=0.01;


            if(gamepad1.a)   running = true;
            if(gamepad1.b)   running = false;

            //sleep(50);

            if(running) {
                balServo.setPosition(servoPosition);
            }
            telemetry.addData("Running: ", running);
            telemetry.addData("kp: ", kp);
            telemetry.addData("ki: ", ki);
            telemetry.addData("kd: ", kd);
            telemetry.addData("error: ", error);
            telemetry.addData("distance: ", distance);
            telemetry.addData("setValue: ", setValue);
            telemetry.addData("PID_p_error: ", PID_p_error);
            telemetry.addData("PID_i_error: ", PID_i_error);
            telemetry.addData("PID_d_error: ", PID_d_error);
            telemetry.addData("PID_total: ", PID_total);
            telemetry.addData("servoPosition: ", servoPosition);

            previous_error = error;
            timer_1.reset();
            telemetry.update();

        }
    }

    public double myMap(double value, double inLow, double inHigh, double outLow, double outHigh) {
        return outLow + (outHigh-outLow)*(value - inLow)/(inHigh-inLow);
    }
}
