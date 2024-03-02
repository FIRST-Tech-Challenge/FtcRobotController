package org.firstinspires.ftc.teamcode.Testing.Teleop_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Ranch Work")
public class RanchWork extends LinearOpMode{
    //private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    private DcMotor arm2 = null;

    private Servo servo = null;





    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        arm  = hardwareMap.get(DcMotor.class, "arm");
        arm2  = hardwareMap.get(DcMotor.class, "arm2");
        servo  = hardwareMap.get(Servo.class, "servo");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);




        while(opModeIsActive()){
            double motor;
            double se;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            motor = Range.clip(drive + turn, -1.0, 1.0) ;
            se = Range.clip(drive - turn, -1.0, 1.0) ;

            arm.setPower(motor);
            arm2.setPower(motor);
            servo.setPosition(se);


            // run until the end of the match (driver presses STOP)

        }

    }
}
