package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake implements Component{

    private PIDController controller;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double p = 0.0015, i = 0, d = 0;
    public static double f = 0;

    public int target = 0;

    public int offset =0;

    Servo led;
    private final Servo claw;
    public final Servo wrist, angleLeft, angleRight, position;
    private final DcMotor outtakeSlideLeft;
    private final DcMotor outtakeSlideRight;
    public final DcMotor outtakeSlideEncoder;

    public int slideIncrement = 0;
    public boolean slideFixed = false;

    VoltageSensor voltageSensor;

    private Status status;

    private ElapsedTime elapsedTime = null;

    protected boolean isLiftReady = false;
    protected boolean isScoringDone = false;

    public boolean drivetrainOverride = false;

    public Intake intake;
    public DriveTrain driveTrain;


    Telemetry telemetry;
    Init init;

    enum Mode {
        Sample,
        Specimen
    }

    enum WaitTime{
        Open_Claw (400), CLose_Claw(400)
        , Turn_Wrist(600), BackUp_Robot(400),
        Servo_To_Transfer(300),
        Move_Position(450)
        ;

        private final long time;

        private WaitTime(long time){
            this.time= time;
        }

        public long getTime() {
            return time;
        }
    }

    public enum Status {
        TransferReady(300),
        TransferWait(0),

        ScoreSpecimen(0),
        Wall (0),
        Bucket(0),
        InitWall(0),
        InitAutoSpec(0),
        InitAutoSample(0),

        TransferToBucket_Back(200),
        TransferToBucket_Lift(0),
        TransferToBucket_Move(600),
        AutoLiftToBucket(0),
//        ScoreSample(100),
//        ScoreSampleOpenClaw(200),
        ScoringSampleDone(300),

        TransferToWall_Up(200),
        TransferToWall_Back(600),
        TransferToWall_Final(200),

//        BucketToTransfer_Down(500),
//        BucketToTransfer_Open(200),
        BucketToTransfer_Final(400),

        WallToFront_lift(600),
        WallToFront_move(450),
        WallToFront3 (300),

        WallToTransfer1(1000), WallToTransfer(300), WallToTransfer2(1000),
        CloseClawTransfer(400),

        Specimen_To_Wall(350),
        SpecimenToWall_MoveBack(1300),

        Transfer_To_Wall(500),
        ToBucketClose(400);


        private final long time;

        private Status (long time){
            this.time= time;
        }

        public long getTime() {
            return time;
        }
    }

    public Outtake(Init init, Telemetry telemetry){

        this.init=init;
        this.telemetry=telemetry;

        claw= init.getClaw();
        wrist= init.getWrist();


        this.outtakeSlideLeft =init.getOuttakeSlideLeft();
        this.outtakeSlideRight =init.getOuttakeSlideRight();
        this.outtakeSlideEncoder =init.getOuttakeSlideRight();
        voltageSensor = init.getVoltageSensor();
        angleLeft = init.getAngleLeft();
        angleRight = init.getAngleRight();
        position = init.getPosition();
        led = init.getLed();
        initializeHardware();

    }

    public void setIntake(Intake intake){
        this.intake= intake;
    }

    public void setDriveTrain(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
    }


    public void initializeHardware() {

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        target = 0;

    }

    public void initTeleopWall(){
        position.setPosition(ITDCons.positionBack);
        angleLeft.setPosition(ITDCons.angleBack);
        angleRight.setPosition(ITDCons.angleBack);
        claw.setPosition(ITDCons.clawOpen);
        status= Status.InitWall;
    }

    public void initAutoSpecimen(){
        //position.setPosition(ITDCons.positionInitSpec);
        setAngleServoScore();
        closeClaw();
        wrist.setPosition(ITDCons.wristFront);
        status= Status.InitAutoSpec;
    }

    public void initAutoSample(){
        position.setPosition(ITDCons.positionBack);
        angleLeft.setPosition(ITDCons.angleMiddle);
        angleRight.setPosition(ITDCons.angleMiddle);
        closeClaw();
        status= Status.InitAutoSample;

    }

    public void openClaw() {

        if (status==Status.ScoreSpecimen){
            status= Status.Specimen_To_Wall;
            elapsedTime = null;

        } else if (status == Status.Bucket){
            elapsedTime= null;
            status = Status.ScoringSampleDone;
        } else {
           claw.setPosition(ITDCons.clawOpen);
        }
    }

    public void openClawAuto() {

        claw.setPosition(ITDCons.clawOpen);

    }

    public void wallGrabAuto() {

        if (status==Status.ScoreSpecimen) {
            claw.setPosition(ITDCons.clawOpen);
            moveToPickUpFromWall();
        }
    }

    public void closeClaw() {
        claw.setPosition(ITDCons.clawClose);
        if(status == Status.TransferReady){
            status = Status.CloseClawTransfer;
            elapsedTime = new ElapsedTime();
        }
    }

    public void moveToPickUpFromWall(){
        if (status==Status.ScoreSpecimen) {

            status= Status.Specimen_To_Wall;
            elapsedTime = new ElapsedTime();
        }
        if (status == Status.InitWall){
            target = ITDCons.WallTarget;
            claw.setPosition(ITDCons.clawOpen);

            status= Status.Wall;
        }
        if (status==Status.TransferReady){

            elapsedTime = null;
            status= Status.Transfer_To_Wall;

        }
    }

    public void moveToTransfer(){

        if (status==Status.Wall || status == Status.InitWall){

            setAngleServoScore();
            closeClaw();
            target = ITDCons.TransferTarget;
            elapsedTime = new ElapsedTime();
            status = Status.WallToTransfer1;

           position.setPosition(ITDCons.positionTransfer);

        } else if (status==Status.ScoringSampleDone) {
//            closeClaw();
//            target = ITDCons.intermediateTarget;
//            position.setPosition(ITDCons.positionTransfer);
//            setAngleServoScore();
            status = Status.BucketToTransfer_Final;
        }

    }

    public void moveToTransferTest(){

            position.setPosition(ITDCons.positionTransfer);
            wrist.setPosition(ITDCons.wristFront);
            angleLeft.setPosition(ITDCons.angleTransfer);
            angleRight.setPosition(ITDCons.angleTransfer);
            claw.setPosition(ITDCons.clawOpen);
            target=ITDCons.TransferTarget;

    }


    public void moveToBucket(){
        closeClaw();
        status = Status.TransferToBucket_Back;
    }

    public void score(){

        if (status== Status.Wall){
            scoreSpecimen();
        } else if (status== Status.CloseClawTransfer){
            scoreSample();
        }

    }


    public void scoreSpecimen(){
        if (status==Status.InitAutoSpec){
            status= Status.WallToFront_lift;
            elapsedTime = new ElapsedTime();
        } else {
            status = Status.WallToFront_lift;
            elapsedTime = null;
        }

        target = ITDCons.SpecimenTarget;

    }

    public void scoreSample(){
        if (status == Status.InitAutoSample){
            setOuttakeBack();
            setAngleServoScore();
            status = Status.AutoLiftToBucket;
        } else{
            closeClaw();
            setAngleServoScore();

            status = Status.ToBucketClose;
        }

        elapsedTime = new ElapsedTime();
        target = ITDCons.BucketTarget;
    }

    public void scoreSampleLow(){
        closeClaw(); //should already be closed but just in case
        status = Status.TransferToBucket_Back;
        elapsedTime = new ElapsedTime();
        target = ITDCons.LowBucketTarget;

    }


    protected void moveSlide() {

//frontRight
        int rotatePos = -outtakeSlideEncoder.getCurrentPosition();

//        telemetry.addData("rotatePos",rotatePos);
        double lift = controller.calculate(rotatePos, target-offset);
       // double lift = pid + f;

//        telemetry.addData("lift", lift);

        outtakeSlideLeft.setPower(lift);
        outtakeSlideRight.setPower(lift);

    }

    public int getTarget(){
        return target;
    }

    public void setTarget(int target){
        this.target = target;
    }

    public int getLiftPos(){
        return outtakeSlideEncoder.getCurrentPosition();
    }

    public void releaseSample(){

        claw.setPosition(ITDCons.clawOpen);

    }


    public void update(){
        moveSlide();

        telemetry.addData("status", status);
        telemetry.addData("target", target);

        switch (status){
            case WallToFront_lift:
                if (elapsedTime ==null) {
                    target = ITDCons.SpecimenTarget;
                    setAngleServoToMiddle();
                    elapsedTime = new ElapsedTime();
                }
                if (elapsedTime.milliseconds()> status.getTime() && elapsedTime.milliseconds()< status.getTime()+WaitTime.Move_Position.getTime()){
                    position.setPosition(ITDCons.positionFront);
                }
                if (elapsedTime.milliseconds()> status.getTime()+WaitTime.Move_Position.getTime()){
                    wrist.setPosition(ITDCons.wristFront);
                    setAngleServoScore();
                    elapsedTime = null;
                    status = Status.ScoreSpecimen;
                }

                break;
//            case WallToFront_move:
//                if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
//                    position.setPosition(ITDCons.positionFront);
//
//                    elapsedTime = new ElapsedTime();
//                    status = Status.WallToFront3;
//                }
//                break;
//            case WallToFront3:
//                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){
//                    wrist.setPosition(ITDCons.wristFront);
//                    setAngleServoScore();
//                    elapsedTime = null;
//                    status = Status.ScoreSpecimen;
//                }
//                break;

            case TransferToBucket_Back:
                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){
                    target = ITDCons.BucketTarget;
                    setAngleServoScoreSample();
                    elapsedTime= new ElapsedTime();
                    status = Status.TransferToBucket_Lift;
                    isScoringDone= false;
                    isLiftReady = false;
                }
                break;

            case TransferToBucket_Lift:

                status= Status.Bucket;
                isLiftReady = true;
               //TODO: change from time to vertical slide position
                if (getLiftPos()>ITDCons.BucketTarget-30000){
                    elapsedTime = new ElapsedTime();

//                    status = Status.TransferToBucket_Move;
                    isLiftReady = true;
                }
                break;
            case TransferToBucket_Move:
                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()) {
                    elapsedTime = null;
                    status = Status.Bucket;
                }
                break;

            case AutoLiftToBucket:

                if (getLiftPos()>ITDCons.BucketTarget-500) {
                    isLiftReady = true;
                }
                break;

//            case ScoreSample:
//                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){
//                    elapsedTime=new ElapsedTime();
//                    status = Status.ScoreSampleOpenClaw;
//                    openClaw();
//                }
//                break;
//            case ScoreSampleOpenClaw:
//                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){
//                    driveTrain.drive(0.5);
//                    elapsedTime = new ElapsedTime();
//                    status= Status.ScoringSampleDone;
//                }
//
//                break;

            case ScoringSampleDone:
                if (elapsedTime== null){
                    claw.setPosition(ITDCons.clawOpen);
                    elapsedTime = new ElapsedTime();
                }
                if ( elapsedTime.milliseconds()>WaitTime.Open_Claw.getTime() && elapsedTime.milliseconds()<WaitTime.Open_Claw.getTime()+WaitTime.BackUp_Robot.getTime() ){
                    driveTrain.drive(0.6);
                    drivetrainOverride = true;
                }
                if (elapsedTime.milliseconds()>WaitTime.Open_Claw.getTime()+WaitTime.BackUp_Robot.getTime()){
                    drivetrainOverride= false;
                    driveTrain.drive(0);
                    status= Status.BucketToTransfer_Final;
                    elapsedTime = null;
                }

                break;

            case BucketToTransfer_Final:
                if (elapsedTime==null){
                    closeClaw();
                    target = ITDCons.TransferTarget;
                    position.setPosition(ITDCons.positionTransfer);
                    setAngleServoToMiddle();
                    elapsedTime = new ElapsedTime();
                }
                if (elapsedTime.milliseconds()>status.getTime() && elapsedTime.milliseconds()<WaitTime.Servo_To_Transfer.getTime()+ status.getTime()){
                    wrist.setPosition(ITDCons.wristFront);
                    setAngleServoToTransfer();
                }
                if (elapsedTime.milliseconds()>WaitTime.Servo_To_Transfer.getTime()+status.getTime()){
                    openClawAuto();
                    status= Status.TransferReady;
                    elapsedTime = null;
                }
                break;

            case WallToTransfer1:
                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){

                    position.setPosition(ITDCons.positionTransfer);
                    wrist.setPosition(ITDCons.wristFront);


                    status = Status.WallToTransfer2;
                    elapsedTime= new ElapsedTime();
                }

            case WallToTransfer2:
                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()) {
                    setAngleServoToTransfer();
                    openClawAuto();
                    status = Status.TransferReady;
                }

                break;

//            case BucketToTransfer_Down:
//
//                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime() && outtakeSlideEncoder.getCurrentPosition()<ITDCons.intermediateTarget+500){
//                    elapsedTime = new ElapsedTime();
//                    status = Status.BucketToTransfer_Open;
//
//                    wrist.setPosition(ITDCons.wristFront);
//                    target = ITDCons.TransferTarget;
//                }
//                break;
//

            case Specimen_To_Wall:
                if (elapsedTime ==null){
                    elapsedTime = new ElapsedTime();
                    openClawAuto();
                }
                if (elapsedTime.milliseconds()>WaitTime.Open_Claw.getTime() && elapsedTime.milliseconds()<WaitTime.Turn_Wrist.getTime()){
                    target = ITDCons.intermediateTarget;
                    wrist.setPosition(ITDCons.wristBack);
                    closeClaw();
                    setAngleServoToMiddle();
                }
                if (elapsedTime.milliseconds()>WaitTime.Turn_Wrist.getTime()){
                    elapsedTime =new ElapsedTime();
                    status = Status.SpecimenToWall_MoveBack;
                }

               break;

            case SpecimenToWall_MoveBack:

                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime() ) {
                    target = ITDCons.wallPickupTarget;

                    setAngleServoToBack();
                    openClaw();
                    status= Status.Wall;
                } else if (elapsedTime!=null && elapsedTime.milliseconds()>700){
                    position.setPosition(ITDCons.positionBack);
                }
                break;

            case Transfer_To_Wall:
                if (elapsedTime == null){
                    target = ITDCons.WallTarget;
                    closeClaw();
                    elapsedTime = new ElapsedTime();

                }
                if (elapsedTime.milliseconds()>WaitTime.CLose_Claw.getTime()){
                    setAngleServoToMiddle();
                    wrist.setPosition(ITDCons.wristBack);
                    status = Status.SpecimenToWall_MoveBack;
                    elapsedTime = new ElapsedTime();
                }
                break;

            case TransferReady:

                //put back code when position is consistent
//                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){
//                    if (intake.readyToTransfer()){
//                        //target= ITDCons.transferPickupTarget;
//                        closeClaw();
//                        status= Status.CloseClawTransfer;
//                        elapsedTime = new ElapsedTime();
//                    }
//                }
                break;
            case CloseClawTransfer:
                if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
                    intake.extendForTransfer();
                    setOuttakeBack();
                }
                break;
            case ToBucketClose:
                if (elapsedTime!=null && elapsedTime.milliseconds()>status.getTime()){
                        setOuttakeBack();
                    status = Status.TransferToBucket_Back;
                    elapsedTime= new ElapsedTime();
                }
                break;




        }
    }

    private void setAngleServoToTransfer(){
        angleLeft.setPosition(ITDCons.angleTransfer);
        angleRight.setPosition(ITDCons.angleTransfer);
    }

    private void setAngleServoToMiddle(){
        angleLeft.setPosition(ITDCons.angleMiddle);
        angleRight.setPosition(ITDCons.angleMiddle);
    }

    private void setAngleServoToBack(){
        angleLeft.setPosition(ITDCons.angleBack);
        angleRight.setPosition(ITDCons.angleBack);
    }

    private void setAngleServoScore(){
        angleLeft.setPosition(ITDCons.angleScoreSpec);
        angleRight.setPosition(ITDCons.angleScoreSpec);
    }

    private void setAngleServoScoreSample(){
        angleLeft.setPosition(ITDCons.angleScoreSample);
        angleRight.setPosition(ITDCons.angleScoreSample);
    }

    private void resetSlides(){
        if (!slideFixed) {
            slideIncrement = slideIncrement - 100;
            setTarget(slideIncrement);
            if (voltageSensor.getVoltage() < 9) {
                outtakeSlideLeft.setPower(0);
                outtakeSlideRight.setPower(0);
                outtakeSlideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlideEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideFixed = true;
            }
        }
        if (slideFixed){
            led.setPosition(1);
        }
    }

    private void setOuttakeBack(){
        position.setPosition(ITDCons.positionBack);
    }

    public void setStatus(Status status) {
        this.status = status;
    }

    public boolean isLiftReady(){
        return isLiftReady;
    }

    public boolean isScoringDone(){
        return  isScoringDone;
    }

    public boolean isReadyToPickUp(){
        return  status== Status.Wall;
    }

    public void setLiftReady(boolean liftReady) {
        isLiftReady = liftReady;
    }

    public boolean isReadyForTransfer(){
        return status==Status.TransferReady;
    }

    public Status getStatus(){
        return status;
    }
}
