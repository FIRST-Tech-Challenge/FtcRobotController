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
    private DistanceSensor sensorRange;
    Servo balServo;


    public static double kp = 0.35;
    public static double ki = 0.005;
    public static double kd = 130;
//works on charged rev battery lol. retune if different. Around 13.5 volts = fully charged
    public static double PID_p_error = 0.0;
    public static double PID_i_error = 0.0;
    public static double PID_d_error = 0.0;
    public static double PID_total = 0.0;

    public static double distance = 0.0;
    public static double error = 0.0;
    public static double previous_error = 0.0;
    public static double setValue = 20;

    // public static double a = 0.9;
    //LowPassFilter filter = new LowPassFilter(a);

    boolean running = false;
    double servoPosition = 0.0;

    @Override
    public void runOpMode() {

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

            double currentValue = sensorRange.getDistance(DistanceUnit.CM);
            //double estimate = filter.estimate(currentValue);
            distance = sensorRange.getDistance(DistanceUnit.CM);
            error = distance - setValue;

            if(running) {
                PID_p_error = kp * error;
                PID_d_error = kd * (error - previous_error) / timer_1.milliseconds();
                balServo.setPosition(servoPosition);
                if (-2 < error && error < 2) {
                    PID_i_error = 0;
                    PID_p_error = 0;
                    PID_d_error = 0;
                } else {
                    PID_i_error = ki * error + PID_i_error;
                }
            }

            PID_total = PID_p_error + PID_i_error + PID_d_error;
            servoPosition = myMap(PID_total,-17.0, 10.0, 0.62, 0.0);

            sleep(50);

            telemetry.addData("kp: ", kp);
            telemetry.addData("ki: ", ki);
            telemetry.addData("kd: ", kd);
            telemetry.addData("distance: ", distance);
            telemetry.addData("PID_total: ", PID_total);
            previous_error = error;
            timer_1.reset();
            telemetry.update();

            //telemetry.addData("low pass", estimate);
            //telemetry.addData("a", a);

        }
    }

    public double myMap(double value, double inLow, double inHigh, double outLow, double outHigh) {
        return outLow + (outHigh-outLow)*(value - inLow)/(inHigh-inLow);
    }
}
