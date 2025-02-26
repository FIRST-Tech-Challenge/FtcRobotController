package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.TwoFishDelivery;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Debug Servo", group="AAA")
public class ServoTestOpmode extends LinearOpMode {

    private int servoIndex = 0;
    private String currentServo = "";

    private Gamepad prevGamepad1 = null;
    private ServoController servoController = null;
    private Servo servo = null;



    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {

        gamepad1.copy(prevGamepad1);

        servoController = hardwareMap.get(ServoController.class, "Control Hub");

        if(servoController == null){
            requestOpModeStop();
        }

        waitForStart();

        while (opModeIsActive()){


            TelemetryPacket packet = new TelemetryPacket();

            if(gamepad1.left_trigger <= 0.5){
                servoController.setServoPosition(servoIndex, gamepad1.right_trigger);
            }

            if(gamepad1.right_bumper && !prevGamepad1.right_bumper) servoIndex ++;
            if(gamepad1.left_bumper && !prevGamepad1.left_bumper) servoIndex --;

            if(gamepad1.b && !prevGamepad1.b){
                servoController.pwmDisable();
            }

            telemetry.addData("Currently controlling servo from port: ", servoIndex);
            telemetry.addData("Servo Position: ", servoController.getServoPosition(servoIndex));
            
            telemetry.update();

        }
    }
}
