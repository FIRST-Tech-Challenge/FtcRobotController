package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

@TeleOp(name="MotorTest")
public class MotorTest extends AbstractOpMode {
    DcMotor motor1, motor2;
    Servo servo;

    protected void onInitialize() {
        motor1 = hardwareMap.dcMotor.get("Motor1");
        motor2 = hardwareMap.dcMotor.get("Motor2");
        servo = hardwareMap.servo.get("Servo");
    }

    @Override
    protected void onStart() {
        servo.setPosition(0);
        Utils.sleep(250);
        servo.setPosition(0.2);
        Utils.sleep(250);

        motor1.setPower(1);
        motor2.setPower(-1);


        while(opModeIsActive()) {
        }

    }

    @Override
    protected void onStop() {

    }
}
