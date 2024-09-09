package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MotorMaximumRPMTest extends OpMode {
    public DcMotor motor;
    public int power = 1;
    public ElapsedTime buttonTimer = new ElapsedTime();
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addLine("Press X on the controller to change motor direction. ");
        telemetry.update();
        if (gamepad1.x && buttonTimer.seconds() > 0.3){
          if (power == 1){
            power = -1;
            buttonTimer.reset();
          }
          else {
            power = 1;
            buttonTimer.reset();
          }
        }
        motor.setPower(power);
    }
}
