package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.scale;

/**
 * Created by Jessica on 4/10/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "StressBall" , group = "TOComp")

public class StressBall extends OpMode {

    private final int PULSE_PER_REVOLUTION_NEVEREST40 = 1120;

    private final int PULSE_PER_REVOLUTION_NEVEREST40_OVER_40 = 280;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
//
    private CRServo FeedMotor;

    private Servo TurnServo;

    private CRServo ContinuousShooter;

    private CRServo SmallContinuous;

    public float SERVO_LATCH_UP = (float) 1.0;
    public float SERVO_STOP = (float) 0;

    private float FeedSpeed = (float) 0.05;

    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo


    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("fl_drv");
        frontRight = hardwareMap.dcMotor.get("fr_drv");
        backLeft = hardwareMap.dcMotor.get("bl_drv");
        backRight = hardwareMap.dcMotor.get("br_drv");
//
        //FeedMotor = hardwareMap.crservo.get("FeedMotor");

        ContinuousShooter = hardwareMap.crservo.get("ContinuousShooter");
        SmallContinuous = hardwareMap.crservo.get("SmallContinuous");

        frontLeft.setMode(RUN_WITHOUT_ENCODER);
        frontRight.setMode(RUN_WITHOUT_ENCODER);
        backLeft.setMode(RUN_WITHOUT_ENCODER);
        backRight.setMode(RUN_WITHOUT_ENCODER);

        backLeft.setDirection(FORWARD);
        frontLeft.setDirection(FORWARD);
        frontRight.setDirection(REVERSE);
        backRight.setDirection(REVERSE);

//        FeedMotor.setDirection(FORWARD);

        telemetry.update();
    }

    @Override
    public void init_loop(){
        super.init_loop();
        //FeedMotor.setMode(RUN_TO_POSITION);
    }


    @Override
    public void loop() {

        float speed = -gamepad1.right_stick_y;
        float direction = gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;

        float Magnitude = Math.abs(speed) + Math.abs(direction) + Math.abs(strafe);
        if (Magnitude < 1) {
            Magnitude = 1;
        }
        frontLeft.setPower(scale(speed + direction - strafe, -Magnitude, Magnitude, -1, 1));
        frontRight.setPower(scale(speed - direction + strafe, -Magnitude, Magnitude, -1, 1));
        backLeft.setPower(scale(speed + direction + strafe, -Magnitude, Magnitude, -1, 1));
        backRight.setPower(scale(speed - direction - strafe, -Magnitude, Magnitude, -1, 1));

//        //FeedMotor
//        if (gamepad1.right_bumper && ContinuousShooter.getPower()==1) {
//            //OneFlickRotation(1); //Change MAX
//
//            FeedMotor.setPower(FeedSpeed);
//            FeedMotor.setTargetPosition(FeedMotor.getCurrentPosition() + PULSE_PER_REVOLUTION_NEVEREST40_OVER_40); //Uses 70 percent of power
//            //telemetry.addData("FlickerAfter", /*"%10d",*/ Flicker.getCurrentPosition());
//
//        }
//        else if (gamepad1.left_bumper && ContinuousShooter.getPower()==1) {
//            FeedMotor.setPower(FeedSpeed);
//            FeedMotor.setTargetPosition(FeedMotor.getCurrentPosition() - PULSE_PER_REVOLUTION_NEVEREST40_OVER_40);
//        }
//
//        else  {
//            FeedMotor.setPower(0);
//        }

        //ContinuosBigOl'Servo

        if (gamepad1.a) {
            ContinuousShooter.setPower(1);
        }
        else if (gamepad1.b){
            ContinuousShooter.setPower(0);
        }

        //ContinuosBabyDuboisServo
        if (gamepad1.dpad_up) {
            SmallContinuous.setPower(-1);
        }
        else if (gamepad1.dpad_down) {
            SmallContinuous.setPower(1);
        } else {
            SmallContinuous.setPower(0);
        }



    }

    @Override
    public void stop() {
        super.stop();
        ContinuousShooter.setPower(0);
    }
}
