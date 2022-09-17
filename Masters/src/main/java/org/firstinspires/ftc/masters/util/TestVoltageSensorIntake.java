package org.firstinspires.ftc.masters.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.masters.util.LynxModuleUtil;

import java.util.LinkedList;
import java.util.Queue;
@Disabled
@TeleOp(name="test voltage intake")
public class TestVoltageSensorIntake extends LinearOpMode {

    DcMotor intake;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    VoltageSensor batteryVoltageSensor;
    int MAX_SIZE= 500;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Queue<Double> voltageQueue = new LinkedList<>();
        double sum= 0;
        intake.setPower(1);
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("voltage", batteryVoltageSensor.getVoltage());


            double voltage= batteryVoltageSensor.getVoltage();
            voltageQueue.add(voltage);

            double avg = sum/voltageQueue.size();
            telemetry.addData("avg", avg);
            telemetry.addData("diff", voltage/avg);
            if (voltage/ avg <0.95) {
                sleep(400);
                intake.setPower(0);
                telemetry.addData("detected", true);
            }
            sum= sum + voltage;
            if (voltageQueue.size()>=MAX_SIZE){
                sum =sum - voltageQueue.remove();
            }

            telemetry.update();


        }

    }
}
