package org.firstinspires.ftc.teamcode.Components;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFBreakBeam;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLimitSwitch;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.openftc.easyopencv.LIFO_OpModeCallbackDelegate;

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
  public static double ONE=0.56, TWO=0.58, THREE = 0.60, FOUR = 0.62, FIVE =0.635, STOP_DELAY = 0.8, UPPIES = 0.74, SUPPER_UPIES = 0.84;
  double lastTime =0;
  double reverseTime = -100;
  boolean pixeled = false;
  boolean pixeled1= false;

  boolean intakePath = false;
  int height = 5;

  double startIntakeTime = -100;

  double intakePathTIme = -100;
  double lastHeightTime=-5;
  double TICKS_PER_REV = 200;

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
    if(!isTeleop){
      height = 5;
    }
    lastHeightTime=-5;

    reverseTime=-100;
    pixeled = !isTeleop;

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
    setRawPower(-INTAKE_POWER-100);
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

  /**
   * Sets intake power to REVERSE_POWER, logs that the robot is reversing to general and intake
   * surface level
   */
  public void reverseIntake() {
    LOGGER.setLogLevel(RFLogger.Severity.INFO);
    LOGGER.log("reversing intake, power : " + REVERSE_POWER);
    setRawPower(- REVERSE_POWER);
    IntakeStates.REVERSING.setStateTrue();
  }

  public void uppies(){
    if (IntakeStates.INTAKING.state || IntakeStates.REVERSING.getState() && this.getPower() == 0) {
      intakeServo.setPosition(UPPIES);
      height =1;
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
      LOGGER.log(RFLogger.Severity.FINE, "stopping intake, power : " + 0 + ", " );
      double pos = this.getVelocity()*0.3+this.getCurrentPosition();
      pos%=TICKS_PER_REV;
      if(pos>TICKS_PER_REV/2){
        pos -= TICKS_PER_REV;
      }
    if (abs(pos) < 20) {
      setRawPower(0);
    }
    IntakeStates.STOPPED.setStateTrue();
      uppies();
      pixeled1=false;

    //        }
    //        LOGGER.log("position" + pos);
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
    if(height == p_height-2){
      return false;
    }
    if( IntakeStates.STOPPED.getState()){
      startIntakeTime=BasicRobot.time;
      storPixel=Magazine.pixels;
      height = p_height;
    }
    if(BasicRobot.time - startIntakeTime>8){
      stopIntake();
    }

    if (BasicRobot.time - lastHeightTime > 5) {
      height = p_height;
      setHeight(height);
      lastHeightTime = BasicRobot.time;
    }
    if (BasicRobot.time - lastHeightTime > 2) {
      height--;
      if(height<=p_height-2){
        return false;
      }
      height=max(height,1);
      setHeight(height);
      lastHeightTime = BasicRobot.time;
    }
    if(Magazine.pixels==1&&storPixel==0&&BasicRobot.time - lastHeightTime > 1){
      height = max(1, height - 1);
      if(height<=p_height-2){
        return false;
      }
      setHeight(height);
      lastHeightTime = BasicRobot.time;
    }
    if(Magazine.pixels!=2){
      intake();
    }
    storPixel=Magazine.pixels;
    return true;
  }
  public void setIntakePath(boolean p_intakePath){
    intakePath = p_intakePath;
    intakePathTIme=time;
  }

  public double getIntakePathTIme(){
    return intakePathTIme;
  }



  public void setHeight(int height){
    double autoOff = 0.02;
    if(isTeleop){
      autoOff=0;
    }
    if(height==1){
      intakeServo.setPosition(ONE-autoOff);
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
    LOGGER.log("intake speed:"+ this.getVelocity());
    packet.put("intakespeed", this.getVelocity());
    double power = this.getPower();
    LOGGER.setLogLevel(RFLogger.Severity.FINEST);
    LOGGER.log("intake power:" + power);
    packet.put("intakePos", this.getCurrentPosition());
    for (var i : IntakeStates.values()) {
      if (i.state) packet.put("IntakeState", i.name());
      if(i.state&&i==IntakeStates.INTAKING)intake();
      if(i.state&&i==IntakeStates.STOPPED)stopIntake();
      if(i.state&&i==IntakeStates.REVERSING)reverseIntake();

    }
    if(Magazine.pixels==1){
      pixeled1=true;
    }

    if(Magazine.pixels==2){
      if(!pixeled){
        lastTime = time;
        pixeled=true;
      }
      if (time - lastTime > STOP_DELAY && !stopped && IntakeStates.INTAKING.getState()) {
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
    if(stopped&& time-reverseTime>0.2){
      stopIntake();
      intakePath=false;
    }

  }
}
