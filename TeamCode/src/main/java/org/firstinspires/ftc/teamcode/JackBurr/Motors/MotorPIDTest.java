package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.nio.channels.MulticastChannel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Config
@TeleOp
public class MotorPIDTest extends OpMode {
    public static double kP = 0.8;
    public static double kI = 0.8;
    public static double kD = 0.8;


    public static double out;
    public static double error;
    public static double reference = 100;
    public static double integralSum = 0;
    public static double derivative = 0;
    public static int encoderPosition;
    public static double lastError = 0;
    public ElapsedTime timer = new ElapsedTime();
    public DcMotorEx motor;
    public static PIDCoefficients coefficients = new PIDCoefficients(kP, kI, kD);
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "intakeSlides");
    }

    @Override
    public void loop() {
                // obtain the encoder position
            encoderPosition = motor.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;
            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();
            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());
            out = (kP * error) + (kI * integralSum) + (kD * derivative);
            FtcDashboard.getInstance().getTelemetry().addData("Error: ", error);
            FtcDashboard.getInstance().getTelemetry().addData("Reference: ", reference);
            telemetry.addData("Error: ", error);
            telemetry.addData("Reference: ", reference);
            telemetry.addLine(String.valueOf(motor.getCurrentPosition()));
            motor.setPower(out);
            if(timer.seconds() > 3) {
                lastError = error;
                reference = reference + 100;
                // reset the timer for next time
                timer.reset();

            }
    }
}
