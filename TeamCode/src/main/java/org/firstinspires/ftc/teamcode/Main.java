package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;


@Autonomous(name = "Main")
public class Main extends LinearOpMode {
    //INIT
    DcMotor motor;

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            motor = hardwareMap.get(DcMotor.class, "motor0");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive()) {
                motor.setPower(0.2);
                sleep(2500);
                motor.setPower(0);
                break;
                // useless
            }
        }
    }
}
