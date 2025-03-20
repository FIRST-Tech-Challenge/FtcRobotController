package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TwoFishDelivery;

@TeleOp(name="WorldFish Double Motor Slide Speed Test", group="Z")
public class WorldFishDoubleMotorTest extends LinearOpMode{

    TwoFishDelivery delivery;
    ElapsedTime deliveryTimer;


    @Override
    public void runOpMode() {
        deliveryTimer = new ElapsedTime();
        delivery = new TwoFishDelivery(this, deliveryTimer);

        delivery.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {

            if(Math.abs(gamepad2.right_stick_y) > 0.1){
                delivery.setSlidesPower(gamepad2.right_stick_y);
            }else{
                delivery.setSlidesPower(0);
            }

            telemetry.update();
        }
    }
}
