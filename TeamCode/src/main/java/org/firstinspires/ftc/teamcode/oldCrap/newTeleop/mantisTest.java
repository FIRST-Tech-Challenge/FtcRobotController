package org.firstinspires.ftc.teamcode.oldCrap.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware;

//@TeleOp(name = "Mantis Test", group = "Teleop")
public class mantisTest extends LinearOpMode {
    hardware hardware = new hardware();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){
            mantis();
//            while(hardware.mantis.isBusy()){
//                telemetry.addData("Posiiton", hardware.mantis.getCurrentPosition());
//                telemetry.update();
//            }
        }
    }

    private void initialize(){
        hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
        hardware.mantis.setDirection(DcMotor.Direction.REVERSE);
        hardware.mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hardware.mantis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void mantis(){
        double driveSpeed = gamepad1.right_stick_y;
        //boolean ballCrusher = gamepad1.right_b
        if(gamepad1.right_stick_y > 0) {
            hardware.mantis.setPower(driveSpeed);

        }else if(gamepad1.right_stick_y < 0){
            hardware.mantis.setPower(0.2 * driveSpeed);
        }else{
            hardware.mantis.setPower(0.1);
        }
    }
}
