package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Control+Cascade", group="Linear Opmode")
public class driveAndLinslide extends LinearOpMode {

    private DcMotor motor = hardwareMap.dcMotor.get("motorFrontLeft");//hardware
    private ElapsedTime runtime;
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH};
    linSlide.states state = linSlide.states.LOW;

    private int toggle;//toggle for setting height
    final double modeCD = 0.15;//these two values are for putting a cooldown on switching heights, just in case pushing down the button slightly too long would make it switch heights more than 1 time
    double CDtimer = 0;

    //Encoder positions for each level on linear slide
    final int low = 0;
    final int mid = 1200;
    final int high = 2600;

    public void initialize(){//initialize linearSlide. it assumes the linear slide starts at the lowest state.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);//change it if needed
        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);//gets time
        toggle=0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        initialize();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
//LINSLIDE CODE STARTS HERE
            if(gamepad1.right_bumper&&(runtime.time()-CDtimer)>=modeCD){
                if(toggle==2){
                    toggle=-1;
                }
                toggle+=1;
                switch(toggle){
                    case 0:
                        state= linSlide.states.toLOW;
                        break;
                    case 1:
                        state= linSlide.states.toMID;
                        break;
                    case 2:
                        state = linSlide.states.toHIGH;
                        break;
                }
                CDtimer=runtime.time();
            }
            if(gamepad1.right_trigger==1){
                state= linSlide.states.toLOW;
            }

            switch (state) {
                case LOW:
                    if (motor.getCurrentPosition() != low) {//checks position again to see if overshoot when toLOW ended. state MID and HIGH do the same.
                        state = linSlide.states.toLOW;
                        break;
                    }
                    //code when low goes here
                    break;
                case MID:
                    if (motor.getCurrentPosition() != mid) {
                        state = linSlide.states.toMID;
                        break;
                    }
                    break;
                case HIGH:
                    if (motor.getCurrentPosition() != high) {
                        state = linSlide.states.toHIGH;
                        break;
                    }
                    break;

                case toLOW:
                    if (motor.getCurrentPosition() == low) {
                        state = linSlide.states.LOW;
                    } else {
                        //motor.setPower(PID(low,prevPos,prevTime));
                        motor.setTargetPosition(low);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
                case toMID:
                    if (motor.getCurrentPosition() == mid) {
                        state = linSlide.states.MID;
                    } else {
                        //motor.setPower(PID(mid,prevPos,prevTime));
                        motor.setTargetPosition(mid);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
                case toHIGH:
                    if (motor.getCurrentPosition() == high) {
                        state = linSlide.states.HIGH;
                    } else {
                        //motor.setPower(PID(high,prevPos,prevTime));
                        motor.setTargetPosition(high);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
            }

            //telemetry
            telemetry.addData("motorPos ", motor.getCurrentPosition());
        }

    }
}
