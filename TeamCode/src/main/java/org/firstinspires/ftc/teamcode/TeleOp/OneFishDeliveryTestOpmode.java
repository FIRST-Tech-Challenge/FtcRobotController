package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.TwoFishDelivery;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="OneFishDeliveryDebug", group="AAA")
public class OneFishDeliveryTestOpmode extends LinearOpMode {

    OneFishSampleDelivery oneFishDelivery = null;

    VoltageSensor voltageSensor = null;
    private int servoIndex = 0;
    private String currentServo = "";
    private int targetHeight;



    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {

        oneFishDelivery = new OneFishSampleDelivery(this, true);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.left_stick_y >= 0.1){
                targetHeight += (int)(gamepad1.left_stick_y *= -0.01f);
            }

            TelemetryPacket packet = new TelemetryPacket();

            if(gamepad1.dpad_up) servoIndex = 0;
            if(gamepad1.dpad_right) servoIndex = 1;
            if(gamepad1.dpad_down) servoIndex = 2;
//            if(gamepad1.dpad_left) servoIndex = 3;

            switch (servoIndex){
                case    0:
                    if(gamepad1.left_trigger <= 0.5){
                        oneFishDelivery.setPitch(gamepad1.right_trigger);
                    }

                    if(gamepad1.a){
                        oneFishDelivery.disableServoController();
                    }

                    currentServo = "pitch";
                    break;
                case 1:
                    if(gamepad1.left_trigger <= 0.5){
                        oneFishDelivery.setClawPosition(gamepad1.right_trigger);
                    }
                    currentServo = "claw";
                    break;
            }

//            oneFishDelivery.addServoTelemetry();
            telemetry.addData("Currently controlling ", currentServo + " servo.");
            telemetry.addData("Current Voltage: ", voltageSensor.getVoltage());

//            twoFishDelivery.addSlideTelemetry();

            telemetry.update();

        }
    }
}
