package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;
import org.firstinspires.ftc.teamcode.TwoFishDelivery;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="DeliveryDebug", group="AAA")
public class TwoFishDeliveryTestOpmode extends LinearOpMode {

    TwoFishDelivery twoFishDelivery = null;
    private int servoIndex = 0;
    private String currentServo = "";
    private int targetHeight;
    private boolean isSlidePowered = false;

    private Gamepad prevGamepad1 = null;



    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ElapsedTime deliveryTimer = new ElapsedTime();
    @Override
    public void runOpMode() {

        gamepad1.copy(prevGamepad1);

        twoFishDelivery = new TwoFishDelivery(this, deliveryTimer);


        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.left_stick_y >= 0.1){
                twoFishDelivery.slideTarget += (int)(gamepad1.left_stick_y *= -0.01f);
            }

            if(isSlidePowered){
                twoFishDelivery.PControlPower(0.5);
            }

            if(gamepad1.a && !prevGamepad1.a){
                isSlidePowered = !isSlidePowered;
            }

            TelemetryPacket packet = new TelemetryPacket();

//            if(gamepad1.dpad_up) servoIndex = 0;
//            if(gamepad1.dpad_right) servoIndex = 1;
//            if(gamepad1.dpad_down) servoIndex = 2;
//            if(gamepad1.dpad_left) servoIndex = 3;

            if(gamepad1.right_bumper && !prevGamepad1.right_bumper) servoIndex ++;
            if(gamepad1.left_bumper && !prevGamepad1.left_bumper) servoIndex --;

            switch (servoIndex){
                case    0:
                    if(gamepad1.left_trigger <= 0.5){
                        twoFishDelivery.setPitch(gamepad1.right_trigger);
                    }
                    currentServo = "pitch";
                    break;
                case 1:
                    if(gamepad1.left_trigger <= 0.5){
                        twoFishDelivery.setClawPosition(gamepad1.right_trigger);
                    }
                    currentServo = "claw";
                    break;
                case 2:
                    if(gamepad1.left_trigger <= 0.5){
                        twoFishDelivery.setWrist(gamepad1.right_trigger);
                    }
                    currentServo = "wrist";
            }

            if(gamepad1.b && !prevGamepad1.b){
                twoFishDelivery.resetPWM();
            }

            twoFishDelivery.addServoTelemetry();
            telemetry.addData("Currently controlling ", currentServo + " servo.");

            twoFishDelivery.addSlideTelemetry();

            telemetry.update();

        }
    }
}
