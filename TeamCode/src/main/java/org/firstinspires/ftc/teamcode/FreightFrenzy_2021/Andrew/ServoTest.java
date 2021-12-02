package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.Andrew;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Linear OpMode")
@Disabled

public class ServoTest extends LinearOpMode{
    private Servo Rotate = null;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean releasedDD1 = true;
        boolean releasedDU1 = true;



        while (opModeIsActive()) {
            runtime.reset();


            //////////////GAMEPAD 1//////////////


            if (gamepad1.dpad_up) {
                if (releasedDU1){

                    while(Rotate.getPosition() <= 1.0 && gamepad1.dpad_left) {
                        Rotate.setPosition(Rotate.getPosition()+0.02);
                        sleep(40);
                    }
                    releasedDU1 = false;
                }
            } else if (!releasedDU1){
                releasedDU1 = true;
            }
            if (gamepad1.dpad_down) {
                if (releasedDD1){
                    while(Rotate.getPosition() >= 0.03 && gamepad1.dpad_right) {
                        Rotate.setPosition(Rotate.getPosition()-0.02);
                        sleep(40);
                    }
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }
            telemetry.addData("Servo","Rotate (%.2f)", Rotate.getPosition());
            telemetry.update();

        }
    }


}