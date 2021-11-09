package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_ElementIntake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo armMotor = hardwareMap.get(Servo.class,"armMotor");
        DcMotor elementIntake = hardwareMap.get(DcMotor.class, "elementIntake");
        Servo trapdoor = hardwareMap.get(Servo.class,"trapdoor");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                armMotor.setPosition(0.92); //straight up
            }
            if (gamepad1.x){
                armMotor.setPosition(0.76); //base level
            }
            if (gamepad1.y){
                armMotor.setPosition(0.82); //second level
            }
            if (gamepad1.b){
                armMotor.setPosition(0.87); //third level
            }
            if (gamepad1.right_trigger > 0) {
                elementIntake.setPower(-1); // edit the speed if necessary
            }
            if (gamepad1.right_trigger == 0) {
                elementIntake.setPower(0); // edit the speed if necessary
            }
            if (gamepad1.left_trigger > 0) {
                elementIntake.setPower(1); // edit the speed if necessary
            }
            if (gamepad1.left_trigger == 0) {
                elementIntake.setPower(0); // edit the speed if necessary
            }
            if (gamepad1.left_bumper) {
                trapdoor.setPosition(1);
            }
            if (gamepad1.right_bumper) {
                trapdoor.setPosition(0.3);
            }
        }
    }


}
