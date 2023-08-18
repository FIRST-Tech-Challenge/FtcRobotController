package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketStates.BASKET_CEILING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketStates.BASKET_DROP;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.BasketStates.BASKET_TRANSFER;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.SlidesStates.SLIDES_RETRACTED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.SlidesStates.SLIDES_RETRACTING;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretAAStates.TURRET_FLAT;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretAAStates.TURRET_RAISED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_ROTATED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_ROTATING_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_ROTATING_COUNTER_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.TurretRotationStates.TURRET_STRAIGHT;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.faked;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.checker;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.isBlue;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.resetten;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.retracting;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.startAngle;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments.RFAngleAdjust;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis;


public class Turret {

    private DcMotorEx turret_Rotation = null;
    private DcMotorEx turret_Extension = null;
    private RFAngleAdjust turret_Angle_Control = null;
//    private Servo turret_Angle_Control2 = null;
    private Servo basketArmServo = null;
    private Servo basketActuationServo = null;
    private double lastTime=0, lastServoPos=0,servoDist=0;
    private final double DEG_PER_TICK_MOTOR = 18.0/116.0, DEG_PER_TICK_SERVO = 118.0/270.0/35.0, minDiffTime =.3;
    private final double TICKS_PER_INCH = 955.0/32.0;
    private double MAX_EXTENSION_TICKS = 1060;
    private double MIN_EXTENSION_TICKS = 0;
    int adder = 0;
    private final double MAX_ROTATION_TICKS = 570;
    double flipStart=0;
    private double[] lastTimes = {0,0};
    private final double TORQUE_GEAR_RATIO = 10;
    private final double SPEED_GEAR_RATIO = 10;
    private final double ANGLE_CONTROL_SERVO_TOTAL_DEGREES = 35;
    public static double [][][]turret_saved_positions={{{950,-440,0},{700,-440,0},{0,500,0},{12,-24,15}}};

    boolean hardware_present = true;
    public static boolean servoPos = false;
    boolean servoPos2 = false;
    boolean downCap = false;
    public static double extendPosition=0, rotatePosition=0;
    boolean angleControlling = false, arming = false, basketing = false;
    public static boolean areTeleop = false;



    // initialization of outtakeMotor
    public Turret(LedColor led_bank, boolean isTeleOp, StateMachine checkers){
        checker = checkers;
        areTeleop = isTeleOp;
        if (hardware_present) {
            turret_Angle_Control = new RFAngleAdjust("turretAngle", "turret_Angle_Control", "turret_Angle_Control2", 118.0/270);
            turret_Rotation = (DcMotorEx)op.hardwareMap.dcMotor.get("turret_Rotation");
            turret_Rotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turret_Extension = (DcMotorEx)op.hardwareMap.dcMotor.get("turret_Extension");
            turret_Extension.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            turret_Angle_Control = opMode.hardwareMap.get(Servo.class, "turret_Angle_Control");
            basketArmServo = op.hardwareMap.get(Servo.class, "basketActuationServo");
            basketActuationServo = op.hardwareMap.get(Servo.class, "basketArmServo");
            turret_Rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        basketActuationServo.setPosition(0.58);

        turret_Angle_Control.setPositions(0);
//        turret_Angle_Control2.setPosition(118.0/270);
        turret_Rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_Rotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret_Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_Extension.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        basketArmServo.setPosition(0.0);

        if(!isTeleOp) {
            basketActuationServo.setPosition(0.58);
            turret_Angle_Control.setPositions(0);
            turret_Rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret_Rotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turret_Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret_Extension.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            basketArmServo.setPosition(0.0);
//            turret_Angle_Control.setPosition(0);
//            turret_Angle_Control2.setPosition(118.0/270);
//            turret_Rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turret_Rotation.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            turret_Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turret_Extension.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            turret_Extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            turret_Rotation.setTargetPosition((int)(-85/DEG_PER_TICK_MOTOR));
//            turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Rotation.setPower(0.5);
//            basketActuationServo.setPosition(0.58);
//            basketArmServo.setPosition(0.0);
        }

        op.sleep(1000);
    }
    public void resetExtension(){
        adder+=25;
        adder+=25;
        retracting=true;
    }
    public void plusExtension(){
        adder-=25;
        adder-=25;
        retracting=true;
    }
    public int turret_extension_pos;
    public int turret_rotation_pos;
    public double turret_angle_control_rotations;
    double turret_angle_control_full_rotations; //will update this in teleop loop

    public int getCurrentPosition(DcMotorEx motor){
        return motor.getCurrentPosition()+adder;
    }
    public void updateTurretPositions(){ //pog
        extendPosition = getCurrentPosition(turret_Extension);
        rotatePosition = turret_Rotation.getCurrentPosition();
        if(extendPosition<20){
            checker.setState(SLIDES_RETRACTED,true);
            checker.setState(SLIDES_EXTENDED, false);
        }
//        else{
//            checker.setState(SLIDES_RETRACTED,false);
//        }
        if (abs(rotatePosition) < 10 && !checker.getState(TURRET_ROTATING_CLOCKWISE) && !checker.getState(TURRET_ROTATING_COUNTER_CLOCKWISE)) {
            checker.setState(TURRET_STRAIGHT,true);
        }
        if (extendPosition > 20) {
            checker.setState(SLIDES_EXTENDED,true);
        }
//        else{
//            checker.setState(SLIDES_EXTENDED,false);
//        }
        if (basketArmServo.getPosition() < 0.05) {
            checker.setState(BASKET_ARM_REST,true);
        }
//        else{
//            checker.setState(BASKET_ARM_REST,false);
//        }
        if (basketActuationServo.getPosition() > 0.85) {
            checker.setState(BASKET_TRANSFER,true);
//            checker.setState(BASKET_DROP,false);
//            checker.setState(BASKET_CEILING,false);


        }
        else if (basketActuationServo.getPosition() < 0.45) {
//            checker.setState(BASKET_TRANSFER,false);
            checker.setState(BASKET_DROP,true);
//            checker.setState(BASKET_CEILING,false);

        }
        else{
//            checker.setState(BASKET_TRANSFER,false);
            checker.setState(BASKET_CEILING,true);
//            checker.setState(BASKET_DROP,false);


        }
        if(!checker.getState(SLIDES_EXTENDED)&&!checker.getState(TURRET_RAISED)&&checker.getState(TURRET_STRAIGHT)){
            resetten=true;
            arming = false;
            basketing = false;
            angleControlling = false;
        }
        else{
            resetten=false;
        }
        op.telemetry.addData("extendoPos", extendPosition);
        op.telemetry.addData("rotatePos", rotatePosition);
        op.telemetry.addData("basketDown", checker.getState(BASKET_ARM_REST));
        op.telemetry.addData("basketActuationDown", checker.getState(BASKET_TRANSFER));

    }
    public void aimHigh(){ //pog
        FlipBasketArmToPosition(.45);
        AngleControlRotating(35);
        TurretExtend(17,24,0.8);
        turret_Rotation.setTargetPosition((int) (-65/DEG_PER_TICK_MOTOR));
        turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Rotation.setPower(0.5);
        op.sleep(2000);
        FlipBasketArmToPosition(.55);
        FlipBasketToPosition(0.0);
    }
    public void TurretExtend (double height_inches, double extension_inches, double power) {
        double extension_length = Math.sqrt(pow(height_inches, 2) + pow(extension_inches, 2));
        if (extension_length * TICKS_PER_INCH > MAX_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MAX_EXTENSION_TICKS);
        }
        else {
            turret_Extension.setTargetPosition((int) (extension_length * TICKS_PER_INCH));
        }
        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Extension.setPower(power);
    }

    public void TurretExtendSimple (int extension_inches, double power) {
        if (extension_inches * TICKS_PER_INCH > MAX_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MAX_EXTENSION_TICKS);
        }
        else if (extension_inches * TICKS_PER_INCH < MIN_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MIN_EXTENSION_TICKS);
        }
        else {
            turret_Extension.setTargetPosition((int) (extension_inches * TICKS_PER_INCH));
        }
        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Extension.setPower(power);
    }
    public void runTurretWithoutEncoder(){ //pog
        turret_Extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret_Rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurretRotate (double targetAngle) {
        op.telemetry.addData("newAngle",targetAngle);
        int curPos = (int)rotatePosition;
        if(targetAngle>MAX_ROTATION_TICKS){
            targetAngle = MAX_ROTATION_TICKS;
        }
        if(targetAngle<-MAX_ROTATION_TICKS){
            targetAngle = -MAX_ROTATION_TICKS;
        }
        double dist = targetAngle - curPos;
        if(abs(dist)<20){
            checker.setState(TURRET_ROTATED, true);
            turret_Rotation.setPower(0);
        }
        else {
//            checker.setState(TURRET_ROTATED, false);
            double targetVelocity = pow(abs(dist), 1.4) / 69 * dist / abs(dist) * (100);
            if (targetVelocity < 0) {
//                checker.setState(TURRET_STRAIGHT, false);
                checker.setState(TURRET_ROTATING_COUNTER_CLOCKWISE, true);
            }
            else if (targetVelocity > 0) {
//                checker.setState(TURRET_STRAIGHT, false);
                checker.setState(TURRET_ROTATING_CLOCKWISE, true);
            }
            turret_Rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret_Rotation.setVelocity(targetVelocity-(turret_Rotation.getVelocity()-targetVelocity)/3);
        }

    }
    public void TurretSlotate (double targetAngle) {
        op.telemetry.addData("newAngle",targetAngle);
        int curPos = (int)rotatePosition;
        if(targetAngle>MAX_ROTATION_TICKS){
            targetAngle = MAX_ROTATION_TICKS;
        }
        if(targetAngle<-MAX_ROTATION_TICKS){
            targetAngle = -MAX_ROTATION_TICKS;
        }
        double dist = targetAngle - curPos;
        if(abs(dist)<20){
            checker.setState(TURRET_ROTATED, true);
            turret_Rotation.setPower(0);
        }
        else {
//            checker.setState(TURRET_ROTATED, false);
            double targetVelocity = pow(abs(dist),1.2) / 50 * dist / abs(dist) * (50);
            turret_Rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret_Rotation.setVelocity(targetVelocity-(turret_Rotation.getVelocity()-targetVelocity)/4);
        }

    }

    public void BasketArmFlipLowExtend (double power) {
//        basketArmServo.setPosition(190 * TORQUE_GEAR_RATIO);
//
//        turret_Extension.setTargetPosition(1);
//        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret_Extension.setPower(power);
    }

    public void BasketArmFlipHighExtend (double power) {
//        basketArmServo.setPosition(115 * TORQUE_GEAR_RATIO);
//
//        turret_Extension.setTargetPosition(1);
//        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret_Extension.setPower(power);
    }

    public void TurretAngleControlRotating (double torget_point) { //pog
        double thisTime = op.getRuntime();
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }

//        if (basketDown && turret_Angle_Control.getPosition() < 0.1 && torget_point < turret_Angle_Control.getPosition()) {
//            basketActuationServo.setPosition(0.77);
//        }

        if(thisTime-lastTime>minDiffTime) {
            turret_Angle_Control.setPositions(torget_point);
            lastTime=thisTime;
            servoDist=Math.abs(torget_point-lastServoPos);
            lastServoPos=torget_point;
        }
//        turret_Angle_Control.setPosition(-.5);
//        turret_Angle_Control2.setPosition(.5);
//        op.telemetry.addData("difference", target_point - turret_Angle_Control.getPosition());

    }
    public void AutoAngleControlRotating (double torget_point) { //pog
        torget_point*=DEG_PER_TICK_SERVO;
        double thisTime = op.getRuntime();
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }

        if(thisTime-lastTime>minDiffTime) {
            turret_Angle_Control.setPositions(torget_point);
            lastTime=thisTime;
            servoDist=Math.abs(torget_point-lastServoPos);
            lastServoPos=torget_point;
            op.telemetry.addData("lastTime",lastTime);
            checker.setState(TURRET_RAISED, true);
        }
//        turret_Angle_Control.setPosition(-.5);
//        turret_Angle_Control2.setPosition(.5);
//        op.telemetry.addData("difference", target_point - turret_Angle_Control.getPosition());

    }
    public void AngleControlRotating (double torget_point) {
        torget_point*=DEG_PER_TICK_SERVO;
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }
        turret_Angle_Control.setPositions(torget_point);

//        turret_Angle_Control.setPosition(-.5);
//        turret_Angle_Control2.setPosition(.5);
//        op.telemetry.addData("difference", target_point - turret_Angle_Control.getPosition());

    }
    public void rotateToPosition(double targetAngle){
        turret_Rotation.setTargetPosition((int) (targetAngle/DEG_PER_TICK_MOTOR));
        turret_Rotation.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret_Rotation.setPower(0.5);
    }
    public void TurretSlidesToPosition (double x, double y, double z, double power, boolean retract) { //pog
        updateTurretPositions();
        double v = x;
        x=y;
        y=v;

        double extension_length_flat = Math.sqrt(pow(x, 2) + pow(y, 2));
        if(extension_length_flat ==0){
            extension_length_flat = 0.01;
        }
        double extension_length = Math.sqrt(pow(extension_length_flat, 2) + pow(z, 2));
        if(extension_length_flat*TICKS_PER_INCH<extendPosition){
            extension_length_flat = extendPosition;
        }
        if (extension_length * TICKS_PER_INCH > MAX_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MAX_EXTENSION_TICKS);
        }
        else {
            turret_Extension.setTargetPosition((int) (extension_length * TICKS_PER_INCH));
        }
        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Extension.setPower(power);
        if(retract) {
            op.sleep(200);
        }
        double rotation_angle = Math.atan2(y,x) * (180 / PI);
        turret_Rotation.setTargetPosition((int) (rotation_angle/DEG_PER_TICK_MOTOR));
        turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Rotation.setPower(power);



        double elevation_angle = Math.atan2(z,extension_length_flat) * (180 / PI);

        op.telemetry.addData("angle control angle", elevation_angle / ANGLE_CONTROL_SERVO_TOTAL_DEGREES);
        if(elevation_angle<0){
            elevation_angle=0;
        }
        if(elevation_angle>35){
            elevation_angle=35;
        }
        AngleControlRotating(elevation_angle);
    }

    public void TurretManualRotation (double rotation) {
        if((rotatePosition > MAX_ROTATION_TICKS && rotation > 0) || (rotatePosition< -MAX_ROTATION_TICKS && rotation < 0)) {
            turret_Rotation.setPower(0);
        }
        else {
            turret_Rotation.setPower(rotation/3);
        }
    }

    public void TurretManualExtension (double turret_extension) { //pog
        if (checker.getState(SLIDES_EXTENDED)) {
            if (((extendPosition > MAX_EXTENSION_TICKS && turret_extension >0) || (extendPosition < MIN_EXTENSION_TICKS && turret_extension<0))) {
                turret_Extension.setPower(0);
                op.telemetry.addData("extreme", extendPosition);

            }
            else if(abs(turret_extension)<.2){
                turret_Extension.setPower((0));
                op.telemetry.addData("little", extendPosition);
            }
            else {
                turret_Extension.setPower((turret_extension));
                op.telemetry.addData("tick", "ext", "retr", extendPosition, turret_extension);

            }
        }
        op.telemetry.addData("turtext", extendPosition);

    }
    public void turretExtendo(double whereExtendo){ //pog
        if(whereExtendo>MAX_EXTENSION_TICKS){
            whereExtendo=MAX_EXTENSION_TICKS-5;
        }
        double distance = whereExtendo-extendPosition;
        if(abs(distance)<20){
            turret_Extension.setVelocity(0);
        }
        else {
            turret_Extension.setVelocity(distance/abs(distance)* 4 * (abs(distance) + 50));
        }
        if(abs(distance)<20&&turret_Extension.getVelocity()<100){
            turret_Extension.setVelocity(0);
            faked=true;
        }
        else{
            faked = false;
        }
    }
    public void TurretManualFlip () {
//        if (turret_Angle_Control.getPosition() > 0.5) {
//            turret_Angle_Control.setPosition(0);
//            turret_Angle_Control2.setPosition(0);
//        }
//        else {
//            turret_Angle_Control.setPosition(1);
//            turret_Angle_Control2.setPosition(1);
//        }
    }

    public void FlipBasket (int up) {
        updateTurretPositions();
        checker.setState(BASKET_DROP, true);
        if(checker.getState(BASKET_ARM_REST)&&areTeleop) {
            basketActuationServo.setPosition(0.92);
        }
        else{
            basketActuationServo.setPosition(0.18);
            if(extendPosition>400&&abs(rotatePosition)>100){
                SavePosition(0);
            }
        }
    }
    public void FlipBasketToPosition (double torget) {
//        updateTurretPositions();
        basketActuationServo.setPosition(torget);
    }
    public void capBasket(){
//        if(!downCap) {
//            basketActuationServo.setPosition(0.0);
//            basketArmServo.setPosition(0.9);
//            AutoAngleControlRotating(0);
//            turret_Rotation.setTargetPosition((int) (0/DEG_PER_TICK_MOTOR));
//            turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Rotation.setPower(0.5);
//            downCap=true;
//        }
//        else{
//            FlipBasketArmToPosition(.45);
//            basketActuationServo.setPosition(0.07);
//            AutoAngleControlRotating(35);
//            turret_Extension.setTargetPosition(1500);
//            turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Extension.setPower(1.0);
//            turret_Rotation.setTargetPosition((int) (-55/DEG_PER_TICK_MOTOR));
//            turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Rotation.setPower(0.5);
//            downCap=false;
//
//        }
    }
    public void FlipBasketArmToPosition (double torget) {
        updateTurretPositions();
        basketArmServo.setPosition(torget);
    }
    public void FlipBasketArmLow () {
        if(checker.getState(BASKET_ARM_REST)) {
            basketArmServo.setPosition(0.9);
        }
        else{
            basketArmServo.setPosition(0.00);
        }
        servoPos = !servoPos;
    }
    public void FlipBasketArmHigh () {
        updateTurretPositions();
        if(abs(EncoderChassis.angle)<45- startAngle*isBlue) {
            if (checker.getState(BASKET_ARM_REST)) {
                basketArmServo.setPosition(0.45);
            } else {
                basketArmServo.setPosition(0.01);
            }
        }
        else{
            if (basketArmServo.getPosition()<0.7) {
                basketArmServo.setPosition(0.8);
            } else {
                basketArmServo.setPosition(0.01);
            }
        }
    }

    public void SavePosition (int up) {
        updateTurretPositions();
        if(abs(EncoderChassis.angle)<45+ startAngle) {
            if(!BlackoutRobot.isBall) {
                turret_saved_positions[0][0][1] = rotatePosition*isBlue;
                turret_saved_positions[0][0][0] = extendPosition;
            }
            else{
                turret_saved_positions[0][1][1] = rotatePosition*isBlue;
                turret_saved_positions[0][1][0] = extendPosition;
            }
        }
        else{
            turret_saved_positions[0][2][1]=rotatePosition*isBlue;
            turret_saved_positions[0][2][0]=extendPosition;
        }
    }

    public void UnsavePosition () {
//        turret_saved_positions[1][0] = turret_saved_positions[0][0];
//        turret_saved_positions[1][1] = turret_saved_positions[0][1];
//        turret_saved_positions[1][2] = turret_saved_positions[0][2];
    }

    public void TurretManualElevation (double elevation) { //stick_y
        double position;
        if (elevation < 0) {
            if (turret_Angle_Control.getPosition() >= 0.03) {
                position = turret_Angle_Control.getPosition() - 0.01;
                turret_Angle_Control.setPositions(position);

            }
        }
        else {
            if (turret_Angle_Control.getPosition() <= 0.97) {
                position = turret_Angle_Control.getPosition() + 0.01;
                turret_Angle_Control.setPositions(position);

            }
        }
    }
    public boolean TurretReset (double power) {

        boolean isReset = true;
        double velocity;
        if(abs(startAngle-EncoderChassis.angle)<45) {
            if (checker.getState(SLIDES_EXTENDED)) {
                turret_Extension.setVelocity(-extendPosition / abs(extendPosition) * 4 * (abs(extendPosition) + 200));
//                checker.setState(SLIDES_EXTENDED, false);
                checker.setState(SLIDES_RETRACTING, true);

            } else {
                turret_Extension.setPower(0);
            }
//            checker.setState(SLIDES_RETRACTING, false);
//            checker.setState(SLIDES_RETRACTED, true);

//            checker.getState(SLIDES_RETRACTED) &&
            if (!checker.getState(TURRET_STRAIGHT)) {
                velocity = -rotatePosition / abs(rotatePosition) * (5 * abs(rotatePosition) + 50);
                turret_Rotation.setVelocity(velocity);
                if (velocity < 0) {
                    checker.setState(TURRET_ROTATING_COUNTER_CLOCKWISE, true);
                }
                else if (velocity > 0) {
                    checker.setState(TURRET_ROTATING_CLOCKWISE, true);
                }
            } else {
                turret_Rotation.setVelocity(0);
            }
//            checker.setState(TURRET_ROTATING_CLOCKWISE, false);
//            checker.setState(TURRET_ROTATING_COUNTER_CLOCKWISE, false);
//            checker.setState(TURRET_STRAIGHT, true);

//            &&checker.getState(SLIDES_RETRACTED)
            if (!arming) {
                flipStart=op.getRuntime();
                op.sleep(200);
                basketArmServo.setPosition(0.00);
                checker.setState(BASKET_ARM_REST, true);
                arming = true;
            }


            if (!basketing) {
                basketActuationServo.setPosition(.1);
                checker.setState(BASKET_CEILING, true);
                basketing = true;
            }


//             && checker.getState(SLIDES_RETRACTED)
            if (!angleControlling) {
                turret_Angle_Control.setPositions(0);
                basketActuationServo.setPosition(.92);
                angleControlling = true;
//                checker.setState(TURRET_LOWERING, true);
            }
//            checker.setState(TURRET_LOWERING, false);
            checker.setState(TURRET_FLAT, true);
            if (!checker.getState(SLIDES_EXTENDED)) {
                basketActuationServo.setPosition(0.92);
                checker.setState(BASKET_TRANSFER, true);
            }

        }
        else{

            if (!angleControlling) {
                turret_Angle_Control.setPositions(0);
                angleControlling = true;
//                checker.setState(TURRET_LOWERING, true);
            }
//            checker.setState(TURRET_LOWERING, false);
            checker.setState(TURRET_FLAT, true);

            if (!arming) {
                basketArmServo.setPosition(0.30);
                arming = true;
                flipStart=op.getRuntime();
            }
            if(abs(rotatePosition)<10){
                basketArmServo.setPosition(0.00);
                checker.setState(BASKET_ARM_REST, true);
            }


            if (rotatePosition<10&&op.getRuntime()-flipStart>0.5&&arming) {
                FlipBasketToPosition(0.9);
            }

            if(op.getRuntime()-flipStart>0.5) {
                if (checker.getState(SLIDES_EXTENDED)) {
                    turret_Extension.setVelocity(-extendPosition / abs(extendPosition) * 4 * (abs(extendPosition) + 100));

                    checker.setState(SLIDES_RETRACTING, true);
                } else {
                    turret_Extension.setVelocity(0);
                }
//                checker.setState(SLIDES_RETRACTING, false);
//                checker.setState(SLIDES_RETRACTED, true);

                if (!checker.getState(TURRET_STRAIGHT)) {
                    velocity = -rotatePosition / abs(rotatePosition) * (5 * abs(rotatePosition) + 30);
                    turret_Rotation.setVelocity(velocity);
                    if (velocity < 0) {
                        checker.setState(TURRET_ROTATING_COUNTER_CLOCKWISE, true);
                    }
                    else if (velocity > 0) {
                        checker.setState(TURRET_ROTATING_CLOCKWISE, true);
                    }
                } else {
                    turret_Rotation.setVelocity(0);
                }
//                checker.setState(TURRET_ROTATING_CLOCKWISE, false);
//                checker.setState(TURRET_ROTATING_COUNTER_CLOCKWISE, false);
//                checker.setState(TURRET_STRAIGHT, true);
            }
        }
//        op.telemetry.addData("basketArmRest", checker.getState(StateMachine.States.BASKET_ARM_REST));
//        op.telemetry.addData("basketTransfer", checker.getState(StateMachine.States.BASKET_CIELING));
//        op.telemetry.addData("Extended", checker.getState(StateMachine.States.EXTENDED));
//        op.telemetry.addData("turretStraight", checker.getState(StateMachine.States.TURRET_STRAIGHT));
        if(checker.getState(BASKET_ARM_REST)&&checker.getState(BASKET_TRANSFER)&&!checker.getState(SLIDES_EXTENDED)&&checker.getState(TURRET_STRAIGHT)){
            isReset = false;
            arming = false;
            basketing = false;
            angleControlling = false;
            resetten=true;
        }
        else{
            resetten=false;
        }
        return isReset;
    }
    public void stopTurn(){
        turret_Rotation.setPower(0);
    }
    public void stopExtend(){
        turret_Extension.setPower(0);
    }
    public void TurretStop () {
        turret_Rotation.setPower(0);
        turret_Extension.setPower(0);
    }
}

