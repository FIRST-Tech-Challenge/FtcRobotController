package org.firstinspires.ftc.teamcode;

import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import static com.qualcomm.robotcore.util.Range.clip;
import static com.qualcomm.robotcore.util.Range.scale;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * init servos
 * pidf arm
 * test claw +arm (WONT WORK)
 *AUTONOMOUS (https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback) (https://www.reddit.com/r/FTC/comments/zdr137/autonomous_mode/)(https://firstroboticsbc.org/ftc/ftc-team-resources/centerstage-my-first-autonomous-program/)
 *
 */
@TeleOp(name = "MechanumTest")
public class mechanumTest extends LinearOpMode {

    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx screwLeft;
    private DcMotorEx screwRight;

    private DcMotorEx arm;

    private Servo vertical;

    private Servo clawLeft;

    private Servo clawRight;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        screwLeft = hardwareMap.get(DcMotorEx.class, "screwLeft");
        screwRight = hardwareMap.get(DcMotorEx.class, "screwRight");
        arm  = hardwareMap.get(DcMotorEx.class, "arm");

        // Put initialization blocks here.
        wheelsInit();
        armInit();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Put loop blocks here.
                mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                Lift();
                arm();
                telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
                telemetry.addData("Right Stick X", gamepad1.right_stick_x);
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void wheelsInit(){
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    private void armInit(){
        screwLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        screwRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        screwLeft.setDirection(DcMotor.Direction.FORWARD);
        screwRight.setDirection(DcMotor.Direction.FORWARD);
        screwLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screwRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }
    private void arm(){
        if(gamepad2.left_stick_y>0){
            arm.setPower(1);
        }
        else if(gamepad2.left_stick_y<0){
            arm.setPower(-1);
        }
        else{
            arm.setPower(0);
        }
    }
    private void claw(boolean open, boolean close, double inputX, double inputY){

        private int lmin = 0; //left limit of servo
        private int lmax = 180; //right limit of servo

        private int rmin = 0; //left limit of servo
        private int rmax = 180; //right limit of servo

        clawLeft.scaleRange(lmin,lmax);
        clawRight.scaleRange(rmin,rmax);
        //we may want to do range.clip instead? HOWEVER that would be set power as well as EVERY .setposition
        //  armServo.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));

        if(open){
          clawLeft.setPosition(0);
          clawRight.setPosition(1);
        }
        if(close){
            clawLeft.setPosition(1);
            clawRight.setPosition(0);
        }

        private double manualSpeed = .1;
        //manual open and close of claw
        if(inputX<0){
            //to the right is closing
            clawLeft.setPosition(clawLeft.getPosition()+manualSpeed);
            clawRight.setPosition(clawRight.getPosition()-manualSpeed);
        }
        else if(inputX>0){
            //to the left is opening
            clawLeft.setPosition(clawLeft.getPosition()-manualSpeed);
            clawRight.setPosition(clawRight.getPosition()+manualSpeed);
        }
        else{
            clawLeft.setPosition(clawLeft.getPosition());
            clawRight.setPosition(clawRight.getPosition());
        }

        private double verticalSpeed = 1;
        //vertical movement of claw
        if(inputY>0){
            vertical.setPosition(vertical.getPosition()+verticalSpeed);
        }
        else if(inputY<0){
            vertical.setPosition(vertical.getPosition()-verticalSpeed);
        }
        else{
            vertical.setPosition(vertical.getPosition());
        }

    }
    private void Lift() {
        if (gamepad2.dpad_down) {
            screwLeft.setPower(1);
            screwRight.setPower(1);
        } else if (gamepad2.dpad_up) {
            screwLeft.setPower(-1);
            screwRight.setPower(-1);
        } else {
            screwLeft.setPower(0);
            screwRight.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void mecanum(double LSY, double LSX, double RSX) {
        int Speed = 1600;
        double lx = Math.pow(LSY,3);
        double ly = Math.pow(LSY,3);
        double rx = Math.pow(RSX,3);
        if(LSX != 0 || LSY != 0 || RSX != 0){
            frontRight.setVelocity(Speed*(clip((ly)+lx,-1,1)+rx));
            backRight.setVelocity(Speed*(clip((ly)-lx,-1,1)+rx));
            frontLeft.setVelocity(Speed*(clip((ly)-lx,-1,1)-rx));
            backLeft.setVelocity(Speed*(clip((ly)+lx,-1,1)-rx));
        }
        else{
            frontLeft.setVelocity(0);
            backLeft.setVelocity(0);
            frontRight.setVelocity(0);
            backRight.setVelocity(0);
        }
    }
}
