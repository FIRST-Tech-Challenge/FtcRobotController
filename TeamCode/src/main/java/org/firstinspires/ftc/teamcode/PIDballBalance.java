package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@TeleOp(name = "PID ball balance", group = "Sensor")
//defining class "PIDballBalance"
public class PIDballBalance extends LinearOpMode {

    //hardware
    DistanceSensor sensorRange;
    Servo balServo;


    public static double currentValue = 0.0; //currentValue obtained from Distance Sensor
    public static double setValue = 20; //Desired position from sensor (cm)
    public static double error = 0.0; //setValue - CurrentValue
    public static double previous_error = 0.0; //error before current error


    public static double kp = 0.15; //affects sensitivity for proportional controller
    public static double ki = 0.005; //affects sensitivity for integral controller
    public static double kd = 30; //affects sensitivity for derivative controller

    public static double PID_p_error = 0.0; //kp * error
    public static double PID_i_error = 0.0; //Ki * error + Previous Integral correction
    public static double PID_d_error = 0.0; //kd * ((prev. error - current error) / ElapsedTime)
    public static double PID_total = 0.0;

    boolean running = false; //state variable for PID
    double servoPosition = 0.0; //calculated to set servo position

    @Override
    public void runOpMode() {


        //telemetry for FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //hardware mapping our hardware
        sensorRange = hardwareMap.get(DistanceSensor.class, "distSensor");
        balServo = hardwareMap.get(Servo.class, "balServo");

        //timer
        ElapsedTime timer_1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer_1.startTime();

        balServo.setPosition(0.00);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a)   running = true;
            if(gamepad1.b)   running = false;

            currentValue = sensorRange.getDistance(DistanceUnit.CM);
            error = currentValue - setValue;

            if(running) {

                if (-2 < error && error < 2) {
                    PID_i_error = 0;
                    PID_p_error = 0;
                    PID_d_error = 0;
                } else {
                    PID_p_error = kp * error;
                    PID_i_error = ki * error + PID_i_error;
                    PID_d_error = kd * (error - previous_error) / timer_1.milliseconds();

                    PID_total = PID_p_error + PID_i_error + PID_d_error;
                    servoPosition = myMap(PID_total, -17.0, 10.0, 0.62, 0.0);
                    balServo.setPosition(servoPosition);
                }


            }
            sleep(50); //time to allow the servo to settle down

            telemetry.addData("Running: ", running);
            telemetry.addData("kp: ", kp);
            telemetry.addData("ki: ", ki);
            telemetry.addData("kd: ", kd);
            telemetry.addData("error: ", error);
            telemetry.addData("distance: ", currentValue);
            telemetry.addData("setValue: ", setValue);
            telemetry.addData("PID_p_error: ", PID_p_error);
            telemetry.addData("PID_i_error: ", PID_i_error);
            telemetry.addData("PID_d_error: ", PID_d_error);
            telemetry.addData("PID_total: ", PID_total);
            telemetry.addData("servoPosition: ", servoPosition);
            telemetry.update();

            previous_error = error; //the current error for this round will be previous error for next cycle
            timer_1.reset(); //resetting elapsed time

        }
    }
    //to normalize values between in range and out range
    public double myMap(double value, double inLow, double inHigh, double outLow, double outHigh) {
        return outLow + (outHigh-outLow)*(value - inLow)/(inHigh-inLow);
    }
}
