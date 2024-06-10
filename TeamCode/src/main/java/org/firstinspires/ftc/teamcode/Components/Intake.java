package org.firstinspires.ftc.teamcode.Components;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Components.Intake.IntakeStates.REVERSING;
import static org.firstinspires.ftc.teamcode.Components.Magazine.pixels;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFBreakBeam;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLimitSwitch;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

/** Warren Class to contain flipping intake and associated functions */
@Config
public class Intake extends RFMotor {
  RFBreakBeam breakBeam;
  RFLimitSwitch limitSwitch;
  Motor motor;

  RFServo intakeServo;
  double requestTime = 0.0;

  private final double INTAKE_POWER = 1.0;
  private final double REVERSE_POWER = -1.0;

  private boolean full = false;
  private int storPixel=0;

  private boolean stopped = true;
  public static double ONE=0.49, TWO=0.53, THREE = 0.56, FOUR = 0.58, FIVE =0.602, STOP_DELAY = 0.4, UPPIES = 0.9, SUPPER_UPIES = 0.9;
  double lastTime =0;
  double reverseTime = -100;
  boolean pixeled = false;
  boolean pixeled1= false;

  boolean intakePath = false;
  int height = -1;

  double startIntakeTime = -100;

  double intakePathTIme = -100;
  double lastHeightTime=-5;
  double TICKS_PER_REV = 52;

  double curPower=0;

  /** initializes all the hardware, logs that hardware has been initialized */
  public Intake() {
    super("intakeMotor", !isTeleop);
    intakeServo = new RFServo("intakeServo", 1.0);
    super.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LOGGER.setLogLevel(RFLogger.Severity.INFO);
    LOGGER.log("Initializing Intake Motor and intake sensors!");
//    intakeServo.setPosition();
    intakeServo.setFlipTime(0.2);
    intakeServo.setLastTime(-100);
    intakeServo.setPosition(SUPPER_UPIES);
    intakeServo.setLastTime(-100);
    IntakeStates.STOPPED.setStateTrue();
    if(isTeleop){
      height = 5;
    }
    if(!isTeleop){
      height=6;
    }
    lastHeightTime=-5;

    reverseTime=-100;
    pixeled = !isTeleop;
    curPower=0;

    //        breakBeam = new RFBreakBeam();
    //        limitSwitch = new RFLimitSwitch("intakeSwitch");
  }

  /** possible states of intake */
  public enum IntakeStates {
    STOPPED(true),
    INTAKING(false),
    REVERSING(false);

    private boolean state;

    IntakeStates(boolean p_state) {
      state = p_state;
    }

    /** sets current state to true, logs that this state is true in general and intake surface */
    public void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < IntakeStates.values().length; i++) {
          IntakeStates.values()[i].state = false;
        }
        LOGGER.log("Intake state changed to: " + this.name());
        this.state = true;
      }
    }

    public boolean getState() {
      return this.state;
    }
  }

  /**
   * Sets intake power to INTAKE_POWER, logs that the robot is intaking to general and intake
   * surface level
   */
  public void intake() {
    LOGGER.log("starting intake, power : " + INTAKE_POWER);
    if(curPower!=-1){setRawPower(-INTAKE_POWER);curPower=-1;}
    IntakeStates.INTAKING.setStateTrue();
    if (isTeleop) {
      downy();
    }
  }

  public void toggleIntakeHeight(){
    if(height!=5){
      height++;
    }
    else{
      height = 1;
    }
    if(height==1)
      intakeServo.setPosition(ONE);
    if(height==2)
      intakeServo.setPosition(TWO);if(height==3)
      intakeServo.setPosition(THREE);if(height==4)
      intakeServo.setPosition(FOUR);if(height==5)
      intakeServo.setPosition(FIVE);
  }

  public void toggleIntakeHeightDown(){
    if(height!=1){
      height--;
    }
    else{
      height = 5;
    }
    if(height==1)
      intakeServo.setPosition(ONE);
    if(height==2)
      intakeServo.setPosition(TWO);if(height==3)
      intakeServo.setPosition(THREE);if(height==4)
      intakeServo.setPosition(FOUR);if(height==5)
      intakeServo.setPosition(FIVE);
  }
  public double getIntakePower(){
    return curPower;
  }

  /**
   * Sets intake power to REVERSE_POWER, logs that the robot is reversing to general and intake
   * surface level
   */
  public void reverseIntake() {
    LOGGER.setLogLevel(RFLogger.Severity.INFO);
    LOGGER.log("reversing intake, power : " + REVERSE_POWER);
    if(curPower!=1){setRawPower(-REVERSE_POWER);curPower=1;}
    REVERSING.setStateTrue();
  }

  public void uppies(){
    if (IntakeStates.STOPPED.getState() && curPower!=0) {
      intakeServo.setPosition(UPPIES);
      if(isTeleop) {
        height = 1;
      }else
        height=6;
    }
  }
  public void downy(){
    if(intakeServo.getPosition()==UPPIES){
      intakeServo.setPosition(ONE);
    }
  }

  public void superYuppers(){
    intakeServo.setPosition(SUPPER_UPIES);
  }

  /** Sets intake power 0, logs that intake is stopped to general and intake surface level */
  public void stopIntake() {
    IntakeStates.STOPPED.setStateTrue();
    if (curPower != 0) {
      uppies();
      LOGGER.log(RFLogger.Severity.FINE, "stopping intake, power : " + 0 + ", ");
      setRawPower(0);
      IntakeStates.STOPPED.setStateTrue();
      curPower=0;
    }
  }
  public double getStartIntakeTime(){
    return startIntakeTime;
  }

  /**
   * count number of pixels using sensor, log to intake and general surface level when change
   *
   * @return number of pixels in intake
   */
  public int countPixels() {
    int count = 0;
    //        if(limitSwitch.isSwitched()){
    //            count++;
    ////            break beam here
    //            if(true){
    //                count++;
    //            }
    //        }
    //        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
    //        LOGGER.log("Intake.countPixels() : count : "+pixelCount + " --> " + count);
    //        if(count!=pixelCount){
    //            LOGGER.setLogLevel(RFLogger.Severity.INFO);
    //            LOGGER.log("Intake.countPixels() : PIXEL COUNT CHANGED, count : "+pixelCount + "
    // --> " + count);
    //            pixelCount=count;
    //        }
    return count;
  }

  public boolean intakeAutoHeight(int p_height) {
    if (curPower == 0) {
      startIntakeTime = time;
    }
    intake();
    if (p_height != height) {
      this.height = p_height;
      setHeight(p_height);
    }
    return false;
  }
  public void setIntakePath(boolean p_intakePath){
    intakePath = p_intakePath;
    intakePathTIme=time;
  }

  public double getIntakePathTIme(){
    return intakePathTIme;
  }

  public int getHeight(){
    return height;
  }


  public void setHeight(int height){
    double autoOff = -0.00;
    this.height = height;
    if(isTeleop){
      autoOff=0;
    }
    if(height==1){
      intakeServo.setPosition(ONE);
    }
    else if(height ==2){
      intakeServo.setPosition(TWO-autoOff);
    }
    else if(height ==3){
      intakeServo.setPosition(THREE-autoOff);
    }
    else if(height==4){
      intakeServo.setPosition(FOUR-autoOff);
    }
    else{
      intakeServo.setPosition(FIVE-autoOff);
    }
  }
  public boolean intakePath(){
    return intakePath;
  }

  /**
   * updates the state machine, log in general and intake surface updates sensor information,
   * triggers following action to reverse/stop intaking
   */
  public void update() {
//    LOGGER.log("intake speed:"+ this.getVelocity());
//    packet.put("intakespeed", this.getVelocity());
////    double power = this.getPower();
//    LOGGER.setLogLevel(RFLogger.Severity.FINEST);
//    LOGGER.log("intake power:" + power);
    packet.put("intakePos", this.getCurrentPosition());
    for (var i : IntakeStates.values()) {
      if (i.state) packet.put("IntakeState", i.name());
      if(i.state&&i==IntakeStates.INTAKING)intake();
      if(i.state&&i==IntakeStates.STOPPED){stopIntake();Magazine.twoPixelTime=time;pixels = 2;}
      if(i.state&&i== REVERSING)reverseIntake();
    }
    if(pixels==1){
      pixeled1=true;
    }

    if(pixels==2){
      if(!pixeled){
        lastTime = time;
        pixeled=true;
      }
      if (time - lastTime > STOP_DELAY && IntakeStates.INTAKING.getState()) {
        uppies();
        reverseIntake();
        reverseTime = time;
        stopped = true;
      }
    }
    else{
      pixeled = false;
      stopped = false;
    }
    if(REVERSING.getState()&&Magazine.pixels == 2 && time-reverseTime>0.3){
      stopIntake();
      stopped = false;
    }

  }
}
