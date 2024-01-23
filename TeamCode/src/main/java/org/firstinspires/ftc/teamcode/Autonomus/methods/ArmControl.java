package org.firstinspires.ftc.teamcode.Autonomus.methods;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class ArmControl extends LinearOpMode {

    public Servo servo_arm;
    public DcMotor motor_arm;

    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void initHW(OpMode op) {

        servo_arm = op.hardwareMap.get(Servo.class, "servo");
        motor_arm = op.hardwareMap.get(DcMotor.class, "motor_arm");

        motor_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setArm (OpMode op, double pos) {

        op.telemetry.addData("motor_arm", motor_arm.getCurrentPosition());
        op.telemetry.addData("needed_motor_arm", pos);
        op.telemetry.update();

        if (motor_arm.getCurrentPosition() < pos) {

            motor_arm.setPower(-0.8);
            op.telemetry.clear();
            op.telemetry.update();

            while (!isStopRequested() && motor_arm.getCurrentPosition() < pos) {
                op.telemetry.addData("motor_arm", motor_arm.getCurrentPosition());
                op.telemetry.addLine("< pos");
                op.telemetry.update();
            }
        }
        else if (motor_arm.getCurrentPosition() > pos) {

            motor_arm.setPower(0.3);
            op.telemetry.clearAll();
            op.telemetry.update();

            while (!isStopRequested() && (motor_arm.getCurrentPosition() > pos)) {
                op.telemetry.addData("motor_arm", motor_arm.getCurrentPosition());
                op.telemetry.addLine("> pos");
                op.telemetry.update();
            }
        }

        motor_arm.setPower(-0.2);
    }

    public void setCatch(double pos) {
        servo_arm.setPosition(pos);
    }
}
