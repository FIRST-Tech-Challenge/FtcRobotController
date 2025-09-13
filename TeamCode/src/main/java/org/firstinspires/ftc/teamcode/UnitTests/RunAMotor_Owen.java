package org.firstinspires.ftc.teamcode.UnitTests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//This class sets a motor to move when
public class RunAMotor_Owen extends LinearOpMode{
    //The motor that is getting run
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        //Prepare the motor, feed some settings
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Send debug message describing robot status
        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        //Handle waiting a while for startups and shutdown
        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        //run until op mode stops - aka, do this until the game doesn't
        while (opModeIsActive()) {
            //set the motor to move if key pressed
            if (gamepad1.y) motor.setPower(1);
            else motor.setPower(0);

            //refresh debug logger
            telemetry.update();
        }
    }
}
