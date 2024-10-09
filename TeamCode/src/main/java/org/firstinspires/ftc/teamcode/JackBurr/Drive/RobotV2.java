package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Motors.ArmMotorV1;

@TeleOp
public class RobotV2 extends OpMode {
    public RobotV2Config config = new RobotV2Config();

    public DcMotor frontLeft; //PORT 0
    public DcMotor frontRight; //PORT 2
    public DcMotor backLeft; //PORT 1
    public DcMotor backRight; // PORT 3
    public DcMotor slidesMotor; //EXPANSION HUB PORT _
    public DcMotor extraArmMotor; //EXPANSION HUB PORT _
    public Servo intakeServo;
    public int servodirection = 0;
    public DcMotor armMotor;
    public double ARM_POWER = config.ARM_POWER;
    public int MOVEMENT_DISTANCE = config.ARM_MOVEMENT_DISTANCE;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public int SLIDES_DOWN = -223;
    public int SLIDES_LOW_BASKET = -1250;
    public int SLIDES_HIGH_BASKET = -1375;
    public int SLIDESPOS = SLIDES_DOWN;
    public int ARM_DOWN = -200;
    public int ARM_LOW_BASKET= -1204;
    public int ARM_HIGH_BASKET = -1666;
    public int ARMPOS = ARM_DOWN;
    public Servo wrist_servo;

    public enum SystemStates{
        HIGH_BASKET,
        LOW_BASKET,
        DOWN
    }

    public enum WristStates {
        LEFT,
        CENTER,
        RIGHT
    }

    public SystemStates states = SystemStates.DOWN;
    public WristStates wristState = WristStates.CENTER;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, config.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, config.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, config.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, config.BACK_RIGHT);
        slidesMotor = hardwareMap.get(DcMotor.class, config.SLIDES);
        extraArmMotor = hardwareMap.get(DcMotor.class, "arm2");
        slidesMotor = hardwareMap.get(DcMotor.class, "slides");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        wrist_servo = hardwareMap.get(Servo.class, "wrist");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extraArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extraArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extraArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        wristState = WristStates.CENTER;
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
                telemetry.addLine(states.toString());
            }
            else if (states == SystemStates.LOW_BASKET) {
                ARMPOS = ARM_HIGH_BASKET;
                SLIDESPOS = SLIDES_HIGH_BASKET;
                states = SystemStates.HIGH_BASKET;
                telemetry.addLine(states.toString());
            }
            else{
                SLIDESPOS = SLIDES_DOWN;
                ARMPOS = ARM_DOWN;
                states = SystemStates.DOWN;
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
        if(buttonTimer.seconds() > 0.3 && gamepad1.left_bumper){
            switch (wristState){
                case LEFT:
                    break;
                case CENTER:
                    wristState = WristStates.LEFT;
                case RIGHT:
                    wristState = WristStates.CENTER;
            }
        }
        if(buttonTimer.seconds() > 0.3 && gamepad1.right_bumper){
            switch (wristState){
                case LEFT:
                    wristState = WristStates.CENTER;
                case CENTER:
                    wristState = WristStates.LEFT;
                case RIGHT:
                    break;
            }
        }

        run_motors();
        arm_goto(ARMPOS, 1);
        slides_goto(SLIDESPOS);
        move_wrist();
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

    public void slides_goto(int pos){
        slidesMotor.setPower(0.6);
        slidesMotor.setTargetPosition(pos);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("SLIDES TARGET: " +  pos);

    }
    public void arm_goto(int pos, double power){
        telemetry.clearAll();
        armMotor.setPower(power);
        extraArmMotor.setPower(power);
        armMotor.setTargetPosition(pos);
        extraArmMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("ARM TARGET: " +  pos);
        telemetry.addLine("ARM 2 TARGET: " +  pos);
    }

    public void arm_up_by(int ticks) {
        ARMPOS = ARMPOS - ticks;
    }
    public void arm_down_by(int ticks) {
        ARMPOS = ARMPOS + ticks;
    }

    public void move_wrist(){
        switch(wristState){
            case LEFT:
                wrist_servo.setPosition(0);
                break;
            case CENTER:
                wrist_servo.setPosition(0.5);
                break;
            case RIGHT:
                wrist_servo.setPosition(1);
                break;
        }
    }
}
