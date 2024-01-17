package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Ezekiel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class Winch extends LinearOpMode{
    private DcMotor winch;
    private Servo lock;
    private double winchPower = 0;// Whatever power the motor can't lift the robot up.

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor winch = hardwareMap.dcMotor.get("Winch");
        Servo lock = hardwareMap.servo.get("Lock");
        Gamepad g1 = new Gamepad();
        
        waitForStart();
        
        while (opModeIsActive()) {
            if (g1.left_trigger > winchPower) {
                lock.setPosition(45);
                winch.setPower(g1.left_trigger);

            } else if (g1.left_trigger <= winchPower){
                lock.setPosition(0);
                winch.setPower(0);
            }
        }
    }
}
