package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Motors.ArmMotorV1;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class RobotV1 extends OpMode {
    public RobotV1Config config = new RobotV1Config();

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
    public int SLIDES_DOWN = -123;
    public int SLIDES_UP = -750;
    public int SLIDESPOS = SLIDES_DOWN;
    public int ARM_DOWN = -140;
    public int ARM_UP= -1204;
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
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
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

    }

    @Override
    public void loop() {
        run_motors();
        arm_goto(ARMPOS);
        slides_goto(SLIDESPOS);
        telemetry.addLine("ARM POSITION: " + armMotor.getCurrentPosition());
        telemetry.addLine("SLIDES POSITION: " + slidesMotor.getCurrentPosition());
        intakeServo.setPosition(servodirection);
        if(buttonTimer.seconds() > 0.3 && gamepad1.b){
            servodirection = switch_servo_dir();
            buttonTimer.reset();
        }
        if(buttonTimer.seconds() > 0.3 && gamepad1.x){
            if (SLIDESPOS == SLIDES_DOWN){
                SLIDESPOS = SLIDES_UP;
            }
            else {
                SLIDESPOS = SLIDES_DOWN;
            }
            buttonTimer.reset();
        }

        if(buttonTimer.seconds() > 0.3 && gamepad1.y){
            ARMPOS = ARM_UP;
            SLIDESPOS = SLIDES_UP;
            states = SystemStates.HIGH_BASKET;
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
    public void arm_goto(int pos){
        telemetry.clearAll();
        armMotor.setPower(1);
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("ARM TARGET: " +  pos);
    }

    public void arm_up_by(int ticks) {
        ARMPOS = ARMPOS - ticks;
    }
    public void arm_down_by(int ticks) {
        ARMPOS = ARMPOS + ticks;
    }
}
