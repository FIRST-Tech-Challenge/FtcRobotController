package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TwoFishDelivery;

import java.util.ArrayList;
import java.util.List;


@Disabled
public class VoltageDebugger {

    private VoltageSensor voltageSensor = null;
    private LinearOpMode linearOpMode;
    public boolean isDisabled = false;
    private double operatingVoltage;
    private ArrayList<Double> actionVoltages;



    public VoltageDebugger(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }

    private void Initialize(){//4mm
        int initAttempts = 0;
        try {
            voltageSensor = linearOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void recordActionVoltage (){
        actionVoltages.add(voltageSensor.getVoltage());
    }

    public void addActionVoltageTelemetry(){
        linearOpMode.telemetry.addData("Operating Voltage: ", operatingVoltage);
        linearOpMode.telemetry.addLine("Action Voltage List: ");
        linearOpMode.telemetry.addData("", actionVoltages);
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

}
