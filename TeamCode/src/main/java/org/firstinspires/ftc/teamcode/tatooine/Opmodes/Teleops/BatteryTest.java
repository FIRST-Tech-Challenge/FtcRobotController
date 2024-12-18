package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp
public class BatteryTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    boolean stoped = false;
    double stopTime = 0;

    private VoltageSensor myControlHubVoltageSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor a = hardwareMap.get(DcMotor.class, "1");
        DcMotor b = hardwareMap.get(DcMotor.class, "2");
        DcMotor c = hardwareMap.get(DcMotor.class, "3");
        DcMotor d = hardwareMap.get(DcMotor.class, "4");
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if (myControlHubVoltageSensor.getVoltage() > 10 && !stoped) {
                a.setPower(1);
                b.setPower(1);
                c.setPower(1);
                d.setPower(1);

            }
            else if (!stoped) {
                stoped = true;
                stopTime = timer.time(TimeUnit.MINUTES);
                a.setPower(0);
                b.setPower(0);
                c.setPower(0);
                d.setPower(0);
            }
            if (stoped) {
                   //if the battery run for less than 3 minutes print the battery is dead
                    telemetry.addData("Battery is dead", stopTime<3);
            }
            telemetry.addLine("Battery Voltage: " + myControlHubVoltageSensor.getVoltage() + " V");
            telemetry.addData("stopTime",stopTime );
            telemetry.addData("timerSec", timer.time(TimeUnit.SECONDS));
            telemetry.addData("timerMin", timer.time(TimeUnit.MINUTES));
            telemetry.update();
        }
    }
}
