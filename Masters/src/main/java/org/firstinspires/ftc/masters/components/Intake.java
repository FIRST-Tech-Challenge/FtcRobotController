package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    PIDController pidController;

    public static double p = 0.003, i = 0, d = 0.00001;

    Init init;

    Telemetry telemetry;
    Servo intakeLeft;
    Servo intakeRight;
    DcMotor intakeMotor;
    DcMotor extendo;
    RevColorSensorV3 colorSensor;
    RevTouchSensor touchSensor;
    DigitalChannel breakBeam;
    Servo led;
    Servo pusher;
    Outtake outtake;
    ITDCons.Color allianceColor;


    public static double RETRACT_POWER = -0.6;
    public static double EXTEND_POWER = 0.6;
    public static double INTAKE_POWER = -1;
    public static double EJECT_POWER = 0.7;

    public enum Status{
        TRANSFER(0), DROP(0), INIT(0), MOVE_TO_TRANSFER(500), EXTEND_TO_HUMAN(0), EJECT(1000), NEUTRAL(0);
        private final long time;
        Status(long time){
            this.time= time;
        }
        public long getTime() {
            return time;
        }
    }

    ElapsedTime elapsedTime = null;
    int tempTarget =-1;

    public  Status status;

    private int target;

    private ITDCons.Color color;

    public Intake(Init init, Telemetry telemetry){
        this.telemetry = telemetry;
        this.init = init;

        intakeLeft = init.getIntakeLeft();
        intakeRight= init.getIntakeRight();
        intakeMotor = init.getIntake();
        extendo = init.getIntakeExtendo();
        colorSensor = init.getColor();
        breakBeam = init.getBreakBeam();
        led = init.getLed();
        pusher = init.getPusherServo();
        initializeHardware();
        status = Status.INIT;
        color= ITDCons.Color.unknown;
    }

    public void setAllianceColor(ITDCons.Color color){
        allianceColor = color;
    }

    public void initStatusTeleop(){
        status = Status.TRANSFER;
        moveIntakeToTransfer();
    }

    public void pickupSample(){
        color= ITDCons.Color.unknown;
        dropIntake();
        startIntake();
        status=Status.DROP;
    }

    public void setOuttake(Outtake outtake){
        this.outtake= outtake;
    }


    public void initializeHardware() {

        pidController = new PIDController(p,i,d);
        target =0;

    }

    public void startIntake (){
        intakeMotor.setPower(INTAKE_POWER);
    }


    public void ejectIntake(){
        intakeMotor.setPower(EJECT_POWER);
        elapsedTime= new ElapsedTime();
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }

    public void retractSlide() {

        target =0;
    }

    public void retractSlideHalf(){
        target = ITDCons.MaxExtension/2;
    }

    public void extendSlideHumanPlayer(){
        target = ITDCons.MaxExtension;
        dropIntake();
        status= Status.EXTEND_TO_HUMAN;

    }

    public void extendSlideHalf() {

        target = ITDCons.MaxExtension/2;
//        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendo.setPower(EXTEND_POWER);
    }

    public void extendSlideMax(){
        if (status==Status.TRANSFER) {
            target = ITDCons.MaxExtension;
        } else {
            moveIntakeToTransfer();
            elapsedTime = new ElapsedTime();
            tempTarget= ITDCons.MaxExtension;
        }
    }

    public void retractExtensionFully(){
        target = ITDCons.MinExtension;
    }

    public void stopExtendo(){
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setPower(0);
    }

//    public void extendSlide(int position){
//            extendo.setTargetPosition(position);
//            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            extendo.setPower(EXTEND_POWER);
//    }

    public boolean extendoSlideIsBusy(){
        return extendo.isBusy();
    }


    public double getExtensionPosition(){
        return extendo.getCurrentPosition();
    }

    public void intakeintake(){
        intakeLeft.setPosition(ITDCons.intakeintakearm);
        intakeRight.setPosition(ITDCons.intakeintakechain);
    }


    public void dropIntake(){
        intakeLeft.setPosition(ITDCons.intakeArmDrop);
        intakeRight.setPosition(ITDCons.intakeChainDrop);
    }

    public void transferIntake(){
        intakeLeft.setPosition(ITDCons.intakeArmTransfer);
        intakeRight.setPosition(ITDCons.intakeChainTransfer);
    }

    public  void intakeToNeutral(){
        intakeLeft.setPosition(ITDCons.intakeArmNeutral);
        intakeRight.setPosition(ITDCons.intakeChainNeutral);
    }

    public void moveIntakeToTransfer(){
        intakeLeft.setPosition(ITDCons.intakeArmTransfer);
        intakeRight.setPosition(ITDCons.intakeChainTransfer);
        status = Status.MOVE_TO_TRANSFER;
        elapsedTime = new ElapsedTime();
    }

    public void openGate(){
        pusher.setPosition(ITDCons.gateOpen);
    }

    public void closeGate(){
        pusher.setPosition(ITDCons.gateClose);
    }

    public void update(){

//frontRight
            int extendoPos = extendo.getCurrentPosition();

//            telemetry.addData("extendoPos",extendoPos);
            double pid = pidController.calculate(extendoPos, target);

//            telemetry.addData("extendo PID",pid);

            extendo.setPower(pid);


//            if (status == Status.MOVE_TO_TRANSFER && elapsedTime!=null && elapsedTime.milliseconds()>500){
//                status = Status.TRANSFER;
//                elapsedTime= null;
//                target =0;
//            }


            switch (status){
                case EXTEND_TO_HUMAN:
                    if (extendoPos>ITDCons.MaxExtension-50) {
                        intakeMotor.setPower(ITDCons.intakeEjectSpeed);
                        status= Status.EJECT;
                        elapsedTime = new ElapsedTime();
                    }
                    break;
                case EJECT:
                    if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
                        intakeMotor.setPower(0);
                        intakeToNeutral();
                        retractSlideHalf();
                        elapsedTime=null;
                    }
                    break;
                case DROP: //get samples from submersible

                    if (!breakBeam.getState()){
                        //read color
                        checkColor();
                        intakeMotor.setPower(0);
                        if (color!= ITDCons.Color.unknown && color!= ITDCons.Color.yellow && color!=allianceColor){
                            ejectIntake();
                        } else if (color== ITDCons.Color.yellow){
                            moveIntakeToTransfer();
                        } else if (color == allianceColor){
                            intakeToNeutral();
                            status= Status.NEUTRAL;
                            if (target<ITDCons.MaxExtension){
                                target = 0;
                            } else {
                                target = ITDCons.MaxExtension/2;
                            }
                        }

                    } else{
                        led.setPosition(ITDCons.off);
                        color = ITDCons.Color.unknown;
                    }
                    break;
                case MOVE_TO_TRANSFER:
                    if (elapsedTime!=null && elapsedTime.milliseconds()> status.getTime()){
                        ejectIntake();
                        status = Status.EJECT;
                    }
                    break;
            }

    }

    public void setTarget(int target){
        this.target= target;
    }

    public void led(){
        led.setPosition(1);
    }


    public void checkColor(){

        if (colorSensor.red()>colorSensor.blue() && colorSensor.red()> colorSensor.green()){
            led.setPosition(ITDCons.red);
            color = ITDCons.Color.red;
        }
        else if (colorSensor.green()>colorSensor.blue() && colorSensor.green()>colorSensor.red()){
            led.setPosition(ITDCons.yellow);
            color = ITDCons.Color.yellow;
        } else if (colorSensor.blue()>colorSensor.green() && colorSensor.blue()>colorSensor.red()){
            led.setPosition(ITDCons.blue);
            color = ITDCons.Color.blue;
        } else {
            led.setPosition(ITDCons.off);
            color = ITDCons.Color.unknown;
        }

    }

    public ITDCons.Color getColor(){
        return color;
    }

    public Status getStatus(){
        return status;
    }


}
