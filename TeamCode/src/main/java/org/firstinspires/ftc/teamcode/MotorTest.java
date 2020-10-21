package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MotorTest", group="Linear Opmode")
@Disabled
public class MotorTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorA;
    //declare variables

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorA  = hardwareMap.get(DcMotor.class, "motora");
        //names in code -> names in config.

        motorA.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        // \/run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean motorOn;
            //variables for calculation of power

            motorOn = gamepad1.a;

            if(motorOn){
                motorA.setPower(1);
            } else{
                motorA.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "motorA (%.2f)", motorOn);
            telemetry.update();
            //display:elapsed time from init. & motor power
        }
    }
}
