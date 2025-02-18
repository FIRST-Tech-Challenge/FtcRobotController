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

    public static double p = 0.04, i = 0, d = 0;

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


    public static double RETRACT_POWER = -0.6;
    public static double EXTEND_POWER = 0.6;
    public static double INTAKE_POWER = -1;

    public enum Status{
        TRANSFER(0), DROP(0), INIT(0), MOVE_TO_TRANSFER(0), EXTEND_TO_HUMAN(0), EJECT(200);
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

    public void initStatusTeleop(){
        status = Status.TRANSFER;
        moveIntakeToTransfer();
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

    public void reverseIntake(){
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void ejectIntake(){intakeMotor.setPower(INTAKE_POWER/2);}

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

    public void extendSlide() {

        if (target < ITDCons.MaxExtension){
            target = target - 50;
        }
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

            if (!breakBeam.getState()){
                //read color
                checkColor();
                intakeMotor.setPower(0);

            }

            if (status == Status.MOVE_TO_TRANSFER && elapsedTime!=null && elapsedTime.milliseconds()>500){
                status = Status.TRANSFER;
                elapsedTime= null;
                if (tempTarget!=-1){
                    target = tempTarget;
                    tempTarget=-1;
                }
            }


            switch (status){
                case EXTEND_TO_HUMAN:
                    if (extendoPos>ITDCons.MaxExtension-50) {
                        intakeMotor.setPower(ITDCons.intakeEjectSpeed);
                        status= Status.EJECT;
                        elapsedTime = new ElapsedTime();
                    }
                    break;
                case EJECT:
                    if (elapsedTime.milliseconds()> status.getTime()){
                        intakeMotor.setPower(0);
                        intakeToNeutral();
                        retractSlideHalf();
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
        if (colorSensor.getRawLightDetected()>ITDCons.blueMin && colorSensor.getRawLightDetected()<ITDCons.blueMax){
            led.setPosition(ITDCons.blue);
        } else if (colorSensor.getRawLightDetected()>ITDCons.redMin && colorSensor.getRawLightDetected()<ITDCons.redMax){
            led.setPosition(ITDCons.red);
        } else if (colorSensor.getRawLightDetected()>ITDCons.yellowMin && colorSensor.getRawLightDetected()<ITDCons.yellowMax){
            led.setPosition(ITDCons.yellow);
        } else {
            led.setPosition(ITDCons.off);
        }
    }

    public ITDCons.Color getColor(){
        return color;
    }


}
