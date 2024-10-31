package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mantis Test", group = "Teleop")
public class mantisTest extends LinearOpMode {
    private DcMotor mantis;
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
        mantis = hardwareMap.get(DcMotor.class, "mantis");
        mantis.setDirection(DcMotor.Direction.FORWARD);
    }
    private void mantis(){
        if(gamepad1.right_trigger > 0){
            mantis.setPower(driveSpeed);
        }else if(gamepad1.left_trigger > 0){
            mantis.setPower(-0.5);
        }
    }
}
