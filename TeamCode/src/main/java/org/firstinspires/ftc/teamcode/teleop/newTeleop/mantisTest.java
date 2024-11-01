package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "Mantis Test", group = "Teleop")
public class mantisTest extends LinearOpMode {
    hardware hardware = new hardware();
    final double driveSpeed = 0.75;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){
            mantis();
        }
    }

    private void initialize(){
        hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
        hardware.mantis.setDirection(DcMotor.Direction.FORWARD);
    }
    private void mantis(){
        if(gamepad1.right_trigger > 0){
            hardware.mantis.setPower(driveSpeed);
        }else if(gamepad1.left_trigger > 0){
            hardware.mantis.setPower(driveSpeed * -0.5);
        }else{
            hardware.mantis.setPower(0.1);
        }
    }
}
