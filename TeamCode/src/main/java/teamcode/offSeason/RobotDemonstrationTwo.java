package teamcode.offSeason;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

import static java.lang.Math.PI;

@TeleOp(name="Robot Demo 2")
public class RobotDemonstrationTwo extends LinearOpMode {

    private static final double WHEEL_RADIUS = 3.54331;
    private static final double TICKS_PER_REV = 288;

    Servo servo;
    DcMotor motor;
    private double encoderInchesTravelled;

    @Override
    public void runOpMode() throws InterruptedException {
            motor = hardwareMap.dcMotor.get("Demo Motor");
            servo = hardwareMap.servo.get("Demo Servo");
            encoderInchesTravelled = 1;
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            waitForStart();
            while(opModeIsActive()){
                if(gamepad1.left_trigger > 0.3){
                    motor.setPower(-gamepad1.left_trigger);
                    if(encoderInchesTravelled > 0) {
                        encoderInchesTravelled *= -1.0;
                    }
                }else if(gamepad1.right_trigger > 0.3){
                    motor.setPower(gamepad1.right_trigger);
                    if(encoderInchesTravelled < 0) {
                        encoderInchesTravelled *= -1.0;
                    }
                }else if(gamepad1.a){
                    if(encoderInchesTravelled > 0){
                        encoderInchesTravelled++;
                    }else{
                        encoderInchesTravelled--;
                    }
                    Utils.sleep(250);
                }else if(gamepad1.x){
                    travelDistance(encoderInchesTravelled, motor.getPower());
                    encoderInchesTravelled = 1;
                }else if(gamepad1.dpad_left){
                    if(encoderInchesTravelled > 0){
                        encoderInchesTravelled += 12;
                    }else{
                        encoderInchesTravelled -= 12;
                    }
                    Utils.sleep(250);
                }else{
                    motor.setPower(0);
                }
                servo.setPosition(gamepad1.left_stick_x * 0.1 + 0.3);
                telemetry.addData("Motor Power", motor.getPower());
                telemetry.addData("encoder distance travel", encoderInchesTravelled);
                telemetry.update();
            }

    }


    private double getSign(double num){
        if(num < 0){
            return -1;
        }else{
            return 1;
        }
    }


    private void travelDistance(double inches, double power) {
        telemetry.clear();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int)((TICKS_PER_REV / (WHEEL_RADIUS * 2 * Math.PI)) * inches);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motor.getTargetPosition() - motor.getCurrentPosition() < 50 && opModeIsActive()){
            motor.setPower(power);
            telemetry.addData("motor travel ticks", motor.getCurrentPosition());
            telemetry.update();

        }
        motor.setPower(0);

    }
}
