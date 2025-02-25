package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Config
public class DeliveryPOnly extends OpMode {
    public DcMotor motor1;
    public DcMotor motor2;
    public double power = 0.2;
    public static double LIFT_SYNC_KP = 0.001;
    public int leftTargetPosition = -5000;
    public int rightTargetPosition = 5000;
    public int LIFT_POSITION_TOLERANCE = 10;
    public double error = 0;
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "deliverySlideL");
        motor2 = hardwareMap.get(DcMotor.class, "deliverySlideR");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(leftTargetPosition);
        motor2.setTargetPosition(rightTargetPosition);
        boolean isOnTarget = false;
        while (!isOnTarget)
        {
            error = Math.abs(motor1.getCurrentPosition()) - Math.abs(motor2.getCurrentPosition());
            double differentiatePower = error * LIFT_SYNC_KP;
            motor1.setPower(Range.clip(power + differentiatePower, -1.0, 1.0));
            motor2.setPower(Range.clip(power - differentiatePower, -1.0, 1.0));
            isOnTarget = Math.abs(leftTargetPosition - motor1.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE &&
                    Math.abs(rightTargetPosition - motor2.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;
        }
        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }
}
