package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristServoTest;

@TeleOp
public class TeleOpV1 extends OpMode {
    //MOTORS====================================================================================================================
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    //ENUMS=====================================================================================================================
    public enum SystemStatesV1 {
        START,
        HOVER_OVER_SAMPLE,
        ARM_UP,
        READY_FOR_DELIVERY,
        DELIVER_UP,
        ERROR
    }
    public enum SlidesState {
        IN,
        OUT
    }
    public SystemStatesV1 state;
    public SlidesState slidesState;
    //HARDWARE==================================================================================================================
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public DifferentialV2 differentialV2 = new DifferentialV2();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public GrippersV1 grippers = new GrippersV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public WristAxonV1 wrist = new WristAxonV1();
    public DeliverySlidesV1 deliverySlides = new DeliverySlidesV1();
    //TIMERS====================================================================================================================
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime grippersTimer = new ElapsedTime();
    public ElapsedTime deliveryArmTimer = new ElapsedTime();
    public ElapsedTime deliveryGrippersTimer = new ElapsedTime();
    public ElapsedTime diffTimer = new ElapsedTime();
    public ElapsedTime deliveryTimer = new ElapsedTime();
    public ElapsedTime thisStateTimer = new ElapsedTime();
    //BOOLEANS==================================================================================================================
    public boolean deliveryGrippersClosed = false;
    public boolean diffTimerIsReset = false;
    public boolean deliveryTimerIsReset = false;
    public boolean grippersClosed = false;
    public boolean slidesReset = false;
    public boolean delivered = false;
    //VARIABLES=================================================================================================================
    public double timeNeeded = 0;
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
        deliveryAxon.init(hardwareMap);
        deliveryGrippers.init(hardwareMap, telemetry);
        wrist.init(hardwareMap);
        deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
        wrist.setPosition(constants.WRIST_CENTER);
        deliverySlides.init(hardwareMap);
        intakeSlides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesState = SlidesState.IN;
        //===========================================================================================================
    }

    @Override
    public void start(){
        buttonTimer.reset();
        deliveryArmTimer.reset();
        deliveryGrippersTimer.reset();
        diffTimer.reset();
        wrist.setPosition(constants.WRIST_CENTER);
    }

    @Override
    public void loop() {
        //MECANUM_DRIVE====================================================================================================
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.right_stick_x; //This is reversed for our turning
        drive(y, x, rx);
        //SYSTEM_STATES====================================================================================================
        if(state == SystemStatesV1.ARM_UP){
            timeNeeded = 3;
        }
        else if(state == SystemStatesV1.DELIVER_UP){
            timeNeeded = 2;
        }
        else if(state == SystemStatesV1.READY_FOR_DELIVERY){
            timeNeeded = 3;
        }
        else {
            timeNeeded = 0.3;
        }
        if(gamepad1.x && buttonTimer.seconds() > 0.3) {
            if(thisStateTimer.seconds() > timeNeeded) {
                state = nextState();
                if (state == SystemStatesV1.ARM_UP) {
                    grippersTimer.reset();
                }
                buttonTimer.reset();
                thisStateTimer.reset();
            }
        }
        switch (state) {
            case START:
                if(!delivered) {
                    deliveryTimer.reset();
                    delivered = true;
                }
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                if(deliveryTimer.seconds() > 2) {
                    if(!slidesReset) {
                        deliverySlides.runLeftSlideToPosition(0, 0.8);
                        deliverySlides.runRightSlideToPosition(0, 0.8);
                        slidesReset = true;
                    }
                    deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                    differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                    differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                    intakeSlides.runToPosition(constants.INTAKE_MOTOR_IN, 0.5);
                    grippers.setPosition(constants.GRIPPERS_CLOSE);
                    grippersClosed = true;
                }
                break;
            case HOVER_OVER_SAMPLE:
                if (gamepad1.left_bumper && buttonTimer.seconds() > 0.35){
                    wrist.moveLeft(0.2);
                    buttonTimer.reset();

                }
                else if (gamepad1.right_bumper && buttonTimer.seconds() > 0.35){
                    wrist.moveRight(0.2);
                    buttonTimer.reset();
                }
                grippers.setPosition(constants.GRIPPERS_OPEN);
                grippersClosed = false;
                intakeSlides.runToPosition(constants.INTAKE_MOTOR_OUT, 0.5);
                differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                break;
            case ARM_UP:
                if(grippersTimer.seconds() > 0.5) {
                    differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                    differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                }
                if(grippersTimer.seconds() > 1){
                    grippers.setPosition(constants.GRIPPERS_CLOSE);
                    if(!diffTimerIsReset) {
                        diffTimer.reset();
                        diffTimerIsReset = true;
                    }
                    if(diffTimer.seconds() > 0.6) {
                        intakeSlides.runToPosition(constants.INTAKE_MOTOR_ALL_THE_WAY_IN, 1);
                    }
                }
                else {
                    grippers.setPosition(constants.GRIPPERS_GRAB);
                }
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                break;
            case READY_FOR_DELIVERY:
                diffTimerIsReset = false;
                //Move delivery arm to sample
                intakeSlides.runToPosition(constants.INTAKE_MOTOR_ALL_THE_WAY_IN, 1);
                if(!deliveryGrippersClosed) {
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                }
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                if(!deliveryTimerIsReset){
                    deliveryGrippersTimer.reset();
                }
                if(deliveryGrippersTimer.seconds() <  0.3) {
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    deliveryGrippersClosed = true;
                }
                break;
            case DELIVER_UP:
                deliverySlides.runLeftSlideToPosition(-2933, 0.8);
                deliverySlides.runRightSlideToPosition(2965, 0.8);
                if(deliveryGrippersTimer.seconds() >  0.3) {
                    grippers.setPosition(constants.GRIPPERS_OPEN);
                }
                deliveryGrippersClosed = false;
                deliveryAxon.setPosition(constants.DELIVERY_UP);
                slidesReset = false;
                delivered = false;
                break;
        }
        telemetry.addLine("STATE: " + state.name());
        telemetry.addLine("\t Intake grippers position: " + grippers.getPosition());
        telemetry.addLine("\t Intake slides power: " + intakeSlides.getPower());
        telemetry.addLine("\t Intake slides position: " + intakeSlides.getCurrentPosition());


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
                return SystemStatesV1.DELIVER_UP;
            case DELIVER_UP:
                return SystemStatesV1.START;
            default:
                return SystemStatesV1.ERROR;
        }
    }
    public void setDiffPosition(double position){
        differentialV2.setTopRightServoPosition(position);
        differentialV2.setTopLeftServoPosition(position);
    }
}
