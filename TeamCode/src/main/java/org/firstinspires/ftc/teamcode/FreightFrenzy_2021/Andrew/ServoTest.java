package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.Andrew;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Testing")


public class ServoTest extends LinearOpMode{
    private CRServo Rotate = null;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        Rotate = hardwareMap.get(CRServo.class, "Rotate");
        Rotate.setDirection(CRServo.Direction.FORWARD);


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
                    Rotate.setDirection(CRServo.Direction.FORWARD);
                    Rotate.setPower(0.5);
                    releasedDU1 = false;
                }
            } else if (!releasedDU1){
                Rotate.setPower(0);
                releasedDU1 = true;
            }
            if (gamepad1.dpad_down) {
                if (releasedDD1){
                    Rotate.setDirection(CRServo.Direction.REVERSE);
                    Rotate.setPower(0.5);
                    releasedDD1 = false;
//                    while(Rotate.getPosition() >= 0.03 && gamepad1.dpad_right) {
//                        Rotate.setPosition(Rotate.getPosition()-0.02);
//                        sleep(40);
//                    }
//                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                Rotate.setPower(0);
                releasedDD1 = true;
            }
            telemetry.addData("Rotate speed","Rotate (%.2f)", Rotate.getPower());
            telemetry.addData("Rotate direction","Rotate (%.2f)", Rotate.getDirection());
            telemetry.update();

        }
    }


}