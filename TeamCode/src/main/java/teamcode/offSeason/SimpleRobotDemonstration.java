package teamcode.offSeason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.common.Utils;

@TeleOp(name="Robot Demo 1")
public class SimpleRobotDemonstration extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("Demo Motor");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_down){
                if(motor.getPower() > 0) {
                    motor.setPower(motor.getPower() * -1.0);
                }
            }else if(gamepad1.dpad_up){
                if(motor.getPower() < 0){
                    motor.setPower(motor.getPower() * -1.0);
                }
            }else if(gamepad1.a){
                motor.setPower(motor.getPower() + 0.1);
                Utils.sleep(250);
            }else if(gamepad1.b){
                motor.setPower(motor.getPower() - 0.1);
                Utils.sleep(250);
            }
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}
