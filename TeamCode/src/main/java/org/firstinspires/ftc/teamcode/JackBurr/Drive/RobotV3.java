package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Motors.ArmMotorV1;

@TeleOp
public class RobotV3 extends OpMode {
    public RobotV2Config config = new RobotV2Config();

    public DcMotor frontLeft; //PORT 0
    public DcMotor frontRight; //PORT 2
    public DcMotor backLeft; //PORT 1
    public DcMotor backRight; // PORT 3
    public DcMotor slidesMotor; //EXPANSION HUB PORT _
    public Servo intakeServo;
    public int servodirection = 0;
    public DcMotor armMotor;
    public double ARM_POWER = config.ARM_POWER;
    public int MOVEMENT_DISTANCE = config.ARM_MOVEMENT_DISTANCE;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public int SLIDES_DOWN = -957;
    public int SLIDES_LOW_BASKET = -1662;
    public int SLIDES_HIGH_BASKET = -1662;
    public int SLIDESPOS = SLIDES_DOWN;
    public int ARM_DOWN = -300;
    public int ARM_LOW_BASKET = -1840;
    public int ARM_HIGH_BASKET = -2500;
    public int ARMPOS = ARM_DOWN;

    public enum SystemStates{
        HIGH_BASKET,
        LOW_BASKET,
        DOWN
    }


    public SystemStates states = SystemStates.DOWN;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, config.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, config.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, config.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, config.BACK_RIGHT);
        slidesMotor = hardwareMap.get(DcMotor.class, config.SLIDES);
        slidesMotor = hardwareMap.get(DcMotor.class, "slides");
        intakeServo = hardwareMap.get(Servo.class, "grippers");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setDirection(config.SLIDES_DIRECTION);
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        states = SystemStates.DOWN;
        arm_goto(ARM_DOWN, 0.8, true);
        slides_goto(SLIDES_DOWN, 0.6, false);
    }

    @Override
    public void loop() {
        telemetry.addLine("ARM POSITION: " + armMotor.getCurrentPosition());
        telemetry.addLine("SLIDES POSITION: " + slidesMotor.getCurrentPosition());
        telemetry.addLine("ARM TARGET POSITION: " + ARMPOS);
        telemetry.addLine("SLIDES TARGET POSITION: " + SLIDESPOS);
        intakeServo.setPosition(servodirection);
        if(buttonTimer.seconds() > 0.3 && gamepad1.b){
            servodirection = switch_servo_dir();
            buttonTimer.reset();
        }
        if(buttonTimer.seconds() > 0.3 && gamepad1.x){
            if(states == SystemStates.DOWN) {
                ARMPOS = ARM_LOW_BASKET;
                SLIDESPOS = SLIDES_LOW_BASKET;
                states = SystemStates.LOW_BASKET;
                arm_goto(ARM_LOW_BASKET, 1, false);
                slides_goto(SLIDES_LOW_BASKET, 0.8, false);
                telemetry.addLine(states.toString());
            }
            else if (states == SystemStates.LOW_BASKET) {
                ARMPOS = ARM_HIGH_BASKET;
                SLIDESPOS = SLIDES_HIGH_BASKET;
                states = SystemStates.HIGH_BASKET;
                arm_goto(ARM_HIGH_BASKET, 1, false);
                slides_goto(SLIDES_HIGH_BASKET, 1, false);
                telemetry.addLine(states.toString());
            }
            else{
                SLIDESPOS = SLIDES_DOWN;
                ARMPOS = ARM_DOWN;
                states = SystemStates.DOWN;
                arm_goto(ARM_DOWN, 0.3, false);
                slides_goto(SLIDES_DOWN, 0.8, false);
                telemetry.addLine(states.toString());
            }
            buttonTimer.reset();
        }
        if(buttonTimer.seconds() > 0.3 && gamepad1.dpad_down){
            arm_down_by(100);
            buttonTimer.reset();
        }
        if(buttonTimer.seconds() > 0.3 && gamepad1.dpad_up){
            arm_up_by(100);
            buttonTimer.reset();
        }

        run_motors();
    }
    public void stop(){
        slides_goto(0, 0.6, true);
        arm_goto(0, 0.5, true);
    }

    public void run_motors(){
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        drive(y, x, rx);

    }
    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public int switch_servo_dir(){
        if(servodirection == 1){
            return 0;
        }
        else {
            return 1;
        }
    }

    public void slides_goto(int pos, double power, boolean wait_){
        slidesMotor.setPower(power);
        slidesMotor.setTargetPosition(pos);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("SLIDES TARGET: " +  pos);
        if (wait_){
            while (slidesMotor.isBusy()){
                //wait
            }
        }

    }
    public void arm_goto(int pos, double power, boolean wait){
        telemetry.clearAll();
        armMotor.setPower(power);
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("ARM TARGET: " + pos);
        if(wait){
            while (armMotor.isBusy()){
                //wait
            }
        }
    }
    public void arm_up_by(int ticks) {
        ARMPOS = ARMPOS - ticks;
    }
    public void arm_down_by(int ticks) {
        ARMPOS = ARMPOS + ticks;
    }

}
