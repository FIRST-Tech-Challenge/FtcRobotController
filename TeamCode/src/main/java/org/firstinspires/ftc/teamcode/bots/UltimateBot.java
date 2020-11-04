package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

public class UltimateBot extends YellowBot
{
    private static int SWINGVALUE = 280;
    private static int MIDDLESWINGVALUE = 260;
    public DcMotor wobbleSwing = null;
    private Servo wobbleClaw = null;
    private Servo ringCamera = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;

    private SwingPosition swingPosition = SwingPosition.Init;
    private static int SWING_LIFTUP = 50;
    private static int SWING_GROUND_POS = 280;
    private static int SWING_LIFT_UP_POS = 230;
    private static int SWING_LIFT_HIGH_POS = 130;


    /* Constructor */
    public UltimateBot(){

    }
    @Override
    public void init(LinearOpMode owner, HardwareMap ahwMap, Telemetry telemetry) throws Exception {
        super.init(owner, ahwMap, telemetry);

        try {
            wobbleSwing = hwMap.get(DcMotor.class, "swing");
            wobbleSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleSwing.setDirection(DcMotor.Direction.FORWARD);
            wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleSwing.setPower(0);
            intake = hwMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(0);
            shooter = hwMap.get(DcMotor.class, "shooter");
            shooter.setDirection(DcMotor.Direction.FORWARD);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues with intake. Check the controller config", ex);
        }


        try{
            wobbleClaw = hwMap.get(Servo.class, "claw");
            wobbleClaw.setPosition(1);
        }
        catch (Exception ex) {
            throw new Exception("Issues with wobbleClaw. Check the controller config", ex);
        }

        try{
            ringCamera = hwMap.get(Servo.class, "camera");
            ringCamera.setPosition(1);
        }
        catch (Exception ex) {
            throw new Exception("Issues with ringCamera. Check the controller config", ex);
        }



        telemetry.addData("Init", "Ultimate is ready");
    }


    public int moveWobbleSwing(double speed){
        double power = Range.clip(speed, -1.0, 1.0);

        wobbleSwing.setPower(power);
        int position = wobbleSwing.getCurrentPosition();
        return position;
    }


    public double getClawPosition(){
        return wobbleClaw.getPosition();
    }

    public double getCameraPosition(){
        return ringCamera.getPosition();
    }


    public void moveWobbleClaw (double position) {
        double p = Range.clip(position, -1.0, 1.0);
        wobbleClaw.setPosition(p);
    }

    public void moveRingCamera (double position) {
        double p = Range.clip(position, -1.0, 1.0);
        ringCamera.setPosition(p);
    }

    @BotAction(displayName = "Move Intake", defaultReturn="")
    public void intake() {
        intake.setPower(0.7);
    }

    @BotAction(displayName = "Stop Intake", defaultReturn="")
    public void stopintake() {
        intake.setPower(0);
    }

    @BotAction(displayName = "Move Shooter", defaultReturn="")
    public void shooter() { shooter.setPower(0.9); }

    @BotAction(displayName = "Stop Shooter", defaultReturn="")
    public void stopshooter() {
        shooter.setPower(0);
    }

    @BotAction(displayName = "Close Claw", defaultReturn="")
    public void closeWobbleClaw () {
        wobbleClaw.setPosition(0);
    }

    @BotAction(displayName = "Open Claw", defaultReturn="")
    public void openWobbleClaw () {
        wobbleClaw.setPosition(1);
    }

    @BotAction(displayName = "Close Claw", defaultReturn="")
    public void leftRingCamera () { ringCamera.setPosition(0);
    }

    @BotAction(displayName = "Open Claw", defaultReturn="")
    public void rightRingCamera () {
        ringCamera.setPosition(1);
    }

    @BotAction(displayName = "Init WobbleSwing", defaultReturn="")
    public void backWobbleSwing (){
        if (this.getSwingPosition() == SwingPosition.Init){
            return;
        }
        wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleSwing.setTargetPosition(0);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.6);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        this.swingPosition = SwingPosition.Init;
        wobbleSwing.setPower(0);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @BotAction(displayName = "Place Wobble", defaultReturn="")
    public void forwardWobbleSwing () {

        if (this.getSwingPosition() == SwingPosition.Ground){
            return;
        }
        wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleSwing.setTargetPosition(SWING_GROUND_POS);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.6);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        this.swingPosition = SwingPosition.Ground;
        wobbleSwing.setPower(0);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @BotAction(displayName = "Lift Wobble Up", defaultReturn="")
    public void liftAndHoldWobbleSwing() {
        wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleSwing.setTargetPosition(SWING_LIFT_UP_POS);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.7);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        this.swingPosition = SwingPosition.LiftUp;
        wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleSwing.setPower(0);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleSwing.setPower(-0.005);
    }

    public void liftHighAndHoldWobble(){
        wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleSwing.setTargetPosition(SWING_LIFT_HIGH_POS);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleSwing.setPower(0.7);
        boolean stop = false;
        while (!stop) {
            stop = wobbleSwing.isBusy() == false;
        }
        this.swingPosition = SwingPosition.LiftUp;
        wobbleSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleSwing.setPower(0);
        wobbleSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    @BotAction(displayName = "Green Light", defaultReturn="")
    public void signalOK(){
        getLights().OK();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < 1){

        }
        getLights().none();
    }

    @BotAction(displayName = "signalProblem", defaultReturn="")
    public void shoot(){
        getLights().problem();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < 1){

        }
        getLights().none();
    }

    @BotAction(displayName = "Detect Stack", defaultReturn = "B")
    public AutoDot detectStack(){
        AutoDot target = null;
        RingDetector rf = null;
        try {
            rf = new RingDetector(this.hwMap, this.getLights(), telemetry);
            target = rf.detectRing(2, telemetry, owner);
        }
        finally {
            if (rf != null) {
                rf.stopDetection();
            }
        }
        return target;
    }

    public SwingPosition getSwingPosition() {
        return swingPosition;
    }
}
