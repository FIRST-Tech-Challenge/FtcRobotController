package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.util.ElapsedTime;

/** This class is designed to make using PID controllers easier when programming.
* all this does is takes in constants and the runtime, is given a target number and a current number.
* then calculates what motor power would be needed to get there.
 * */

public class PIDController {
    //Connects with the PIDFController class
  
  // The constants that you need to tune specifically for each system
  private double pConstant,iConstant, dConstant;
  private double iSum=0;

  private double iNormal;
  
  // The motor power (-1.0 ~ 1.0) the PD controller outputs
  private double power;
  public double powerProportional;
  public double powerIntegral;
  public double powerDerivative;
  
  // The target number, NOTE this could be encoder ticks, degrees of rotation, or even inches on the field.
  // This is just a number without a specified unit.
  public double target;
  
  // Some data from the last iteration of the control loop.
  private double lastTime, lastError;
  
  // The runtime instance from the main op-mode
  private ElapsedTime runtime;
  private boolean shouldWrap = true;
  
  private double minWrap = 0.0, maxWrap = 0.0;

  /**
   * This is the constructor for this class, this just assigns the constants from the specific mechanism.
   * @param pConstant - Proportional Constant - used for tuning the Proportional factor.
   * @param iConstant - Integral Constant- used for tuning the Integral factor
   * @param dConstant - Derivative Constant - used for tuning the Derivative factor
   * @param runtime - The runtime instance from the main op-mode
   */
  public PIDController(double pConstant, double iConstant, double dConstant, ElapsedTime runtime) {
    this.pConstant = pConstant;
    this.dConstant = dConstant;
    this.iConstant = iConstant;
    this.runtime = runtime;
    shouldWrap = true;

    iNormal = iConstant;

    // to ensure lastTime is correct for the first iteration of update.
    lastTime = runtime.time();
  }
  public PIDController(double pConstant, double iConstant, double dConstant, ElapsedTime runtime, boolean shouldWrap) {
    this.pConstant = pConstant;
    this.dConstant = dConstant;
    this.iConstant = iConstant;
    this.runtime = runtime;
    this.shouldWrap = shouldWrap;

    iNormal = iConstant;

    // to ensure lastTime is correct for the first iteration of update.
    lastTime = runtime.time();
  }

  public void updatePIDConstants(double pConstant, double iConstant, double dConstant){
    this.pConstant = pConstant;
    this.dConstant = dConstant;
    this.iConstant = iConstant;
  }

  /**
   * This updates the output of the PID controller.
   * @param currentPosition the current number from the motor,
   *                        this is a sensor output that is used to calculate the new error
   */
  public void update(double currentPosition) {
     double deltaTime = (runtime.time()-lastTime);
    // the error of how far you are from where you want to be
    // added wrap to the error equation
    double error = currentPosition - target;
    if (shouldWrap) {
      error = Utlities.wrap(error);
    }
    iSum += error *deltaTime;

    
    // The derivative factor scales down the Proportional factor so that we don't over shoot our target.
    double derivative = (error - lastError) / deltaTime;
    
    // the proportional factor affected by the derivative factor to get our output
    power = -error * pConstant +(-iSum * iConstant) + (-dConstant * derivative);

    if (Math.abs(power) >1){
      iSum -= error * deltaTime;
      power = -error * pConstant  + (-dConstant * derivative);
    }

    powerProportional= -error * pConstant;
    powerIntegral= (-iSum * iConstant);
    powerDerivative= (-dConstant * derivative);
    // to ensure that power is always within our range of -1 - 1
    // if power is - 5 then it is divided by 5 to give us -1
    if (power > 1 || power < -1) power /= Math.abs(power);
    
    // updates the last variables that we need for the next iteration of the loop.
    lastTime+= deltaTime;
    lastError = error;
  }
  
  /**
   * used to update the target of where you want to go
   * @param target the position you want to go to
   */
  public void setTarget(double target) {
    this.target = target;
  }
  
  /**
   * Used for getting the target in the turning code for teleop
   * @return returns the target
   */
  public double getTarget() {
    return target;
  }
  
  public void setWrap(double min, double max){
    minWrap = min;
    maxWrap = max;
  }

  public void setIntegralMode(boolean mode){
    if(mode){
      if (iConstant == 0){
        iSum = 0;
      }
      iConstant = iNormal;
    }else{
      iConstant = 0;
    }
  }

  /**
   * used to get the power output from the update function.
   * @return type double which is the power of the motor, range of -1.0 ~ 1.0
   */
  public double getPower() {
    return power;
  }
    public double getPConstant(){
        return pConstant;
    }
    public double getIConstant(){
        return iConstant;
    }
    public double getDConstant(){
        return dConstant;
    }
}
