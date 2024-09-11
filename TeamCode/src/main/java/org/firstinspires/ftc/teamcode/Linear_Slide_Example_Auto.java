package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Linear Slide Example Auto", group = "Example")
public class Linear_Slide_Example_Auto extends LinearOpMode {

    DcMotor motor;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setMode(STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(RUN_TO_POSITION);
        motor.setZeroPowerBehavior(BRAKE);


        waitForStart();

        boolean goingOut = true;

        motor.setTargetPosition(4250);

        for (double d = 0; d <= 1; d += 0.1) {

            motor.setPower(d);

            while (runtime.seconds() < 1 && opModeIsActive()) {

                if (motor.getCurrentPosition() >= 4000 && goingOut) {
                    motor.setTargetPosition(0);
                    goingOut = false;

                } else if (motor.getCurrentPosition() <= 250 && !goingOut){
                    motor.setTargetPosition(4250);
                    goingOut = true;
                }

                telemetry.addData("Pos >", motor.getCurrentPosition());
                telemetry.addData("Speed >", d);
                telemetry.update();
            }
            runtime.reset();

        }

        motor.setTargetPosition(0);
        while (motor.isBusy() && opModeIsActive())
            idle();
    }
}