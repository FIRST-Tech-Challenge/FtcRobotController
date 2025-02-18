package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.util.Optional;
@Disabled
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
        HOVER_LOW,
        DOWN_ON_SAMPLE,
        ARM_UP,
        READY_FOR_DELIVERY,
        DELIVER_LOW_BASKET,
        GRAB_OFF_WALL,
        HOLD_SAMPLE_OUT,
        LIFT_FROM_WALL,
        DELIVER_HIGH_BAR,
        DELIVER_UP,
        LEVEL_ONE_ASCENT,
        READY_FOR_LEVEL_TWO_ASCENT,
        LEVEL_TWO_ASCENT,
        DROP,
        DROP_HIGH_BAR,
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
    public ElapsedTime highBarTimer = new ElapsedTime();
    public ElapsedTime grippersTimer = new ElapsedTime();
    public ElapsedTime deliveryArmTimer = new ElapsedTime();
    public ElapsedTime deliveryGrippersTimer = new ElapsedTime();
    public ElapsedTime diffTimer = new ElapsedTime();
    public ElapsedTime slidesTimer = new ElapsedTime();
    public ElapsedTime deliveryTimer = new ElapsedTime();
    public ElapsedTime thisStateTimer = new ElapsedTime();
    public ElapsedTime diffTimer2 = new ElapsedTime();
    public ElapsedTime deliveryGrippersTimer2 = new ElapsedTime();
    public ElapsedTime hangTimer01 = new ElapsedTime();
    //BOOLEANS==================================================================================================================
    public boolean deliveryGrippersClosed = false;
    public boolean diffTimerIsReset = false;
    public boolean deliveryTimerIsReset = false;
    public boolean grippersClosed = false;
    public boolean grippersClosed2 = false;
    public boolean slidesReset = false;
    public boolean delivered = false;
    public boolean pickedUpSample = false;
    public boolean grippersOpened = false;
    public boolean droppedSample = false;
    public boolean slowmode = false;
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
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.right_stick_x; //This is reversed for our turning
        if(!slowmode) {
            drive(y, x, rx);
        }
        else {
            driveSlowMode(y,x,rx);
        }
        //SYSTEM_STATES====================================================================================================
        if(state == SystemStatesV1.ARM_UP){
            timeNeeded = 2;
        }
        else if(state == SystemStatesV1.DELIVER_UP){
            timeNeeded = 1;
        }
        else if(state == SystemStatesV1.READY_FOR_DELIVERY){
            timeNeeded = 1;
        }
        else {
            timeNeeded = 0.3;
        }

        if(gamepad1.left_trigger != 0 && state != SystemStatesV1.READY_FOR_LEVEL_TWO_ASCENT && state != SystemStatesV1.LEVEL_TWO_ASCENT && buttonTimer.seconds() > 0.3){
            state = SystemStatesV1.READY_FOR_LEVEL_TWO_ASCENT;
            hangTimer01.reset();
            buttonTimer.reset();
        }
        else if(gamepad1.left_trigger != 0 && state == SystemStatesV1.READY_FOR_LEVEL_TWO_ASCENT && buttonTimer.seconds() > 0.3){
            state = SystemStatesV1.LEVEL_TWO_ASCENT;
            buttonTimer.reset();
        }

        if(gamepad1.x && buttonTimer.seconds() > 0.3) {
            if(thisStateTimer.seconds() > timeNeeded) {
                state = nextStateHighBasket();
                if (state == SystemStatesV1.ARM_UP) {
                    grippersTimer.reset();
                }
                else if(state == SystemStatesV1.DROP_HIGH_BAR){
                    highBarTimer.reset();
                }
                buttonTimer.reset();
                thisStateTimer.reset();
            }
        }


        switch (state) {
            case START:
                slowmode = false;
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                wrist.setPosition(constants.WRIST_CENTER);
                if(deliverySlides.getLeftSlidePosition() != 0) {
                    deliverySlides.runLeftSlideToPosition(0, 1);
                }
                if(deliverySlides.getRightSlidePosition()!= 0) {
                    deliverySlides.runRightSlideToPosition(0, 1);
                }
                slidesReset = true;
                differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                intakeSlides.intakeIn();
                grippers.setPosition(constants.GRIPPERS_CLOSE);
                grippersClosed = true;
                if(gamepad1.b && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.GRAB_OFF_WALL;
                    buttonTimer.reset();
                    break;
                }
                break;
            case GRAB_OFF_WALL:
                slowmode = false;
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
                deliverySlides.runLeftSlideToPosition(0, 0.8);
                deliverySlides.runRightSlideToPosition(0, 0.8);
                deliveryAxon.setPosition(constants.DELIVERY_WALL_PICKUP);
                break;
            case LIFT_FROM_WALL:
                deliverySlides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BAR, 0);
                deliverySlides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BAR, 0);
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.GRAB_OFF_WALL;
                    buttonTimer.reset();
                }
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                break;
            case DELIVER_HIGH_BAR:
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
                deliverySlides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BAR, 0.8);
                deliverySlides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BAR, 0.8);
                deliveryAxon.setPosition(constants.DELIVERY_HIGH_BAR);
                if(gamepad1.dpad_up && buttonTimer.seconds() > 0.3){
                    deliveryAxon.setPosition(deliveryAxon.getPosition() + 0.05);
                    buttonTimer.reset();
                }
                else if(gamepad1.dpad_down && buttonTimer.seconds() > 0.3){
                    deliveryAxon.setPosition(deliveryAxon.getPosition() - 0.05);
                    buttonTimer.reset();
                }
                break;
            case HOVER_LOW:
                slowmode = true;
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
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
                intakeSlides.intakeOut();
                differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_LOW_HOVER);
                differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_LOW_HOVER);
                diffTimer.reset();
                diffTimer2.reset();
                grippersTimer.reset();
                pickedUpSample = false;
                break;
            case HOVER_OVER_SAMPLE:
                slowmode = true;
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.HOVER_LOW;
                    buttonTimer.reset();
                }
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
                intakeSlides.intakeOut();
                differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_HOVER);
                differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_HOVER);
                diffTimer.reset();
                diffTimer2.reset();
                grippersTimer.reset();
                pickedUpSample = false;
                break;
            case DOWN_ON_SAMPLE:
                slowmode = true;
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
                if (gamepad1.left_bumper && buttonTimer.seconds() > 0.35){
                    wrist.moveLeft(0.1);
                    buttonTimer.reset();

                }
                else if (gamepad1.right_bumper && buttonTimer.seconds() > 0.35){
                    wrist.moveRight(0.1);
                    buttonTimer.reset();
                }
                differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_PICKUP);
                differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_PICKUP);
                grippersOpened = false;
                break;
            case ARM_UP:
                slowmode = false;
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
                else if(gamepad1.right_bumper && buttonTimer.seconds() > 0.3){
                    slidesTimer.reset();
                    buttonTimer.reset();
                    state = SystemStatesV1.HOLD_SAMPLE_OUT;
                }
                //TODO: Sam said to make button to release grippers, but this current program can't do that
                if(!pickedUpSample){
                    if(diffTimer2.seconds() > 0.3){
                        grippers.setPosition(constants.GRIPPERS_GRAB);
                    }
                   if (diffTimer2.seconds() > 0.6){
                       pickedUpSample = true;
                   }
                   grippersClosed2 = false;
                }
                else {
                    if(!grippersClosed2) {
                        wrist.setPosition(constants.WRIST_CENTER);
                        if (grippersTimer.seconds() > 0.5) {
                            differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_TRANSFER);
                            differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_TRANSFER);
                        }
                        if (grippersTimer.seconds() > 0.6){
                            grippersTimer.reset();
                            grippersClosed2 = true;
                        }
                    }
                    else {
                        if (grippersTimer.seconds() > 0.1) {
                            grippers.setPosition(constants.GRIPPERS_CLOSE);
                            if (!diffTimerIsReset) {
                                diffTimer.reset();
                                diffTimerIsReset = true;
                            }
                            //if (diffTimer.seconds() > 0.6) {
                            if(differentialV2.rightEncoderIsPast(292) && differentialV2.leftEncoderIsPast(77)){
                                intakeSlides.intakeAllTheWayIn();
                            }
                        } else {
                            grippers.setPosition(constants.GRIPPERS_GRAB);
                        }
                    }
                }
                    deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                deliveryGrippersTimer2.reset();
                deliveryTimerIsReset = false;
                droppedSample = false;
                break;
            case HOLD_SAMPLE_OUT:
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
                intakeSlides.intakeOut();
                differentialV2.setTopRightServoPosition(constants.FRONT_RIGHT_HOVER);
                differentialV2.setTopLeftServoPosition(constants.FRONT_LEFT_HOVER);
                if(slidesTimer.seconds() > 1){
                    grippers.setPosition(constants.GRIPPERS_OPEN);
                }
                break;
            case READY_FOR_DELIVERY:
                slowmode = false;
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
                else if(gamepad1.a && buttonTimer.seconds() > 0.3){
                    if(grippersOpened){
                        state = SystemStatesV1.DELIVER_LOW_BASKET;
                        buttonTimer.reset();
                        grippersOpened = true;
                    }
                }
                else if(gamepad1.circle && buttonTimer.seconds() > 0.3){
                    buttonTimer.reset();
                    state = SystemStatesV1.DELIVER_HIGH_BAR;
                }

                diffTimerIsReset = false;
                //Move delivery arm to sample
                intakeSlides.intakeAllTheWayIn();
                if(!deliveryGrippersClosed) {
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                }
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                if(!deliveryTimerIsReset){
                    deliveryGrippersTimer.reset();
                    deliveryTimerIsReset = true;
                }
                if(deliveryGrippersTimer.seconds() <  0.3) {
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    deliveryGrippersClosed = true;
                    deliveryGrippersTimer2.reset();
                }
                else {
                    grippers.setPosition(constants.GRIPPERS_OPEN);
                    grippersOpened = true;
                }

                deliveryGrippersTimer2.reset();
                break;
            case DELIVER_UP:
                slowmode = false;
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }

                deliverySlides.runLeftSlideToPosition(constants.LEFT_SLIDE_HIGH_BASKET, 0.8);
                deliverySlides.runRightSlideToPosition(constants.RIGHT_SLIDE_HIGH_BASKET, 0.8);
                deliveryGrippersClosed = false;
                deliveryAxon.setPosition(constants.DELIVERY_UP);
                slidesReset = false;
                delivered = false;
                break;
            case DELIVER_LOW_BASKET:
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }

                deliverySlides.runLeftSlideToPosition(constants.LEFT_SLIDE_LOW_BASKET, 0.8);
                deliverySlides.runRightSlideToPosition(constants.RIGHT_SLIDE_LOW_BASKET, 0.8);
                deliveryGrippersClosed = false;
                deliveryAxon.setPosition(constants.DELIVERY_UP);
                slidesReset = false;
                delivered = false;
                break;
            case DROP:
                slowmode = false;
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                deliveryAxon.setPosition(constants.DELIVERY_DROP);
                break;
            case DROP_HIGH_BAR:
                if(gamepad1.y && buttonTimer.seconds() > 0.3){
                    state = SystemStatesV1.START;
                    buttonTimer.reset();
                }
                slowmode = false;
                deliveryAxon.setPosition(constants.DELIVERY_DROP);
                deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                //if(highBarTimer.seconds() > 0.4){
                    //deliverySlides.runRightSlideToPosition();
                //}
                break;
            case READY_FOR_LEVEL_TWO_ASCENT:
                deliverySlides.runLeftSlideToPosition(constants.LEFT_SLIDE_LEVEL_TWO_ASCENT, 0.8);
                deliverySlides.runRightSlideToPosition(constants.RIGHT_SLIDE_LEVEL_TWO_ASCENT, 0.8);
                deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                break;

            case LEVEL_TWO_ASCENT:
                deliverySlides.runLeftSlideToPosition(0, 0.8);
                deliverySlides.runRightSlideToPosition(0, 0.8);
                break;

        }
        telemetry.addLine("STATE: " + state.name());
        telemetry.addLine("\t Intake grippers position: " + grippers.getPosition());
        telemetry.addLine("\t Intake slides power: " + intakeSlides.getPower());
        telemetry.addLine("\t Intake slides position: " + intakeSlides.getCurrentPosition());
        telemetry.addLine("\t Delivery Grippers Timer 02: " + deliveryGrippersTimer2.seconds());
        telemetry.addLine("\t Left Delivery SLide: " +  deliverySlides.getLeftSlidePosition());
        telemetry.addLine("\t Right Delivery SLide: " +  deliverySlides.getRightSlidePosition());


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

    public void driveSlowMode(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower / 2);
        backLeftMotor.setPower(backLeftPower / 2);
        frontRightMotor.setPower(frontRightPower / 2);
        backRightMotor.setPower(backRightPower / 2);
    }

    public SystemStatesV1 nextStateHighBasket(){
        switch (state){
            case START:
                return SystemStatesV1.HOVER_OVER_SAMPLE;
            case GRAB_OFF_WALL:
                return SystemStatesV1.LIFT_FROM_WALL;
            case LIFT_FROM_WALL:
                return SystemStatesV1.DELIVER_HIGH_BAR;
            case DELIVER_HIGH_BAR:
                return SystemStatesV1.DROP_HIGH_BAR;
            case DROP_HIGH_BAR:
                return SystemStatesV1.START;
            case HOLD_SAMPLE_OUT:
                return SystemStatesV1.START;
            case HOVER_OVER_SAMPLE:
                return SystemStatesV1.HOVER_LOW;
            case HOVER_LOW:
                return SystemStatesV1.DOWN_ON_SAMPLE;
            case DOWN_ON_SAMPLE:
                return SystemStatesV1.ARM_UP;
            case ARM_UP:
                return SystemStatesV1.READY_FOR_DELIVERY;
            case READY_FOR_DELIVERY:
                return SystemStatesV1.DELIVER_UP;
            case DELIVER_UP:
                return SystemStatesV1.DROP;
            case DELIVER_LOW_BASKET:
                return SystemStatesV1.DROP;
            case DROP:
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
