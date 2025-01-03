package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;

@TeleOp
public class TeleOpV1 extends OpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public enum SystemStatesV1 {
        START,
        HOVER_OVER_SAMPLE,
        ARM_UP,
        READY_FOR_DELIVERY,
        ERROR
    }
    public SystemStatesV1 state;
    public DifferentialV2 differentialV2 = new DifferentialV2();
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime grippersTimer = new ElapsedTime();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public GrippersV1 grippers = new GrippersV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    @Override
    public void init() {
        //Motors =====================================================================================================
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = SystemStatesV1.START;
        differentialV2.init(hardwareMap, telemetry);
        intakeSlides.init(hardwareMap);
        grippers.init(hardwareMap, telemetry);
        //===========================================================================================================
    }

    @Override
    public void start(){
        buttonTimer.reset();
    }

    @Override
    public void loop() {
        //MECANUM_DRIVE====================================================================================================
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.right_stick_x; //This is reversed for our turning
        drive(y, x, rx);
        //SYSTEM_STATES====================================================================================================
        if(gamepad1.x && buttonTimer.seconds() > 0.3) {
            state = nextState();
            if(state == SystemStatesV1.ARM_UP){
                grippersTimer.reset();
            }
            buttonTimer.reset();
        }
        switch (state) {
            case START:
                differentialV2.topServosUp(true);
                intakeSlides.runToPosition(constants.INTAKE_MOTOR_IN, 0.5);
                grippers.setPosition(constants.GRIPPERS_CLOSE);
                break;
            case HOVER_OVER_SAMPLE:
                grippers.setPosition(constants.GRIPPERS_OPEN);
                intakeSlides.runToPosition(constants.INTAKE_MOTOR_OUT, 0.5);
                differentialV2.topServosDown(true);
                break;
            case ARM_UP:
                grippers.setPosition(constants.GRIPPERS_CLOSE);
                if(grippersTimer.seconds() > 0.3){
                    differentialV2.topServosUp(true);
                    intakeSlides.runToPosition(constants.INTAKE_MOTOR_IN, -0.5);
                }
                break;
            case READY_FOR_DELIVERY:
                //Move delivery arm to sample
                //Close delivery grippers
                break;
        }
        telemetry.addLine("STATE: " + state.name());

    }

    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public SystemStatesV1 nextState(){
        switch (state){
            case START:
                return SystemStatesV1.HOVER_OVER_SAMPLE;
            case HOVER_OVER_SAMPLE:
                return SystemStatesV1.ARM_UP;
            case ARM_UP:
                return SystemStatesV1.READY_FOR_DELIVERY;
            case READY_FOR_DELIVERY:
                return SystemStatesV1.START;
            default:
                return SystemStatesV1.ERROR;
        }
    }
}
