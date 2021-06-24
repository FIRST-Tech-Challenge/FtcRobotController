package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.common.AbstractOpMode;
@Disabled
@TeleOp(name="MotorTest")
public class MotorTest extends AbstractOpMode {
    DcMotor motor1, motor2;


    @Override
    protected void onInitialize() {
        motor1 = hardwareMap.dcMotor.get("TestMotor1");
        motor2 = hardwareMap.dcMotor.get("TestMotor2");
    }

    @Override
    protected void onStart() {
        new Thread(){
            public void run(){
                while(opModeIsActive()){
                    motor1.setPower(gamepad1.left_trigger);
                    motor2.setPower(gamepad1.right_trigger);
                }
            }
        }.start();

        while(opModeIsActive()) {
            telemetry.addData("Motor 1 Power: ", motor1.getPower());
            telemetry.addData("Motor 2 Power: ", motor2.getPower());
            telemetry.update();
        }

    }

    @Override
    protected void onStop() {

    }
}
