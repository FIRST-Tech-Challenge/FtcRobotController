package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utils;

public class AutoSwerve {

  AnalogInput servoInputFL,servoInputFR,servoInputBR,servoInputBL;
  Servo servoFL,servoFR,servoBR,servoBL;
  DcMotor motorFL,motorFR,motorBR,motorBL;
  LinearOpMode opMode;
  static double countsPerRevolution = 537.7;
  static double gearRatio = 1.1;
  static double wheelCircumferenceMeters = (96.0 / 1000.0) * Math.PI;
  static double maxMotorVelocity = 436.0 / 60.0;
  double wheelAngle = 180; // direction wheels are pointing

  private static final double conversionFactor;

  static final double maxDriveSpeedMetersPerSec;
  static final double maxSteerSpeedRadPerSec;

  static {
    conversionFactor = countsPerRevolution * gearRatio / wheelCircumferenceMeters;
    maxDriveSpeedMetersPerSec = (maxMotorVelocity / gearRatio) * wheelCircumferenceMeters;

    double maxSpeedSecondsPer60Degrees = .14 * .863;
    maxSteerSpeedRadPerSec = (2 * Math.PI) / (maxSpeedSecondsPer60Degrees * 6);
  }

  GoBildaPinpointDriver odo;
  public AutoSwerve(LinearOpMode opMode,GoBildaPinpointDriver odo){
    this.opMode = opMode;
    this.odo = odo;
    //FL
    servoInputFL = opMode.hardwareMap.analogInput.get("FLEncoder");
    servoFL = opMode.hardwareMap.servo.get("FLServo");
    motorFL = opMode.hardwareMap.dcMotor.get("FLMotor");
    motorFL.setDirection(DcMotorSimple.Direction.FORWARD);

    //FR
    servoInputFR = opMode.hardwareMap.analogInput.get("FREncoder");
    servoFR = opMode.hardwareMap.servo.get("FRServo");
    motorFR = opMode.hardwareMap.dcMotor.get("FRMotor");
    motorFR.setDirection(DcMotorSimple.Direction.FORWARD);

    //BR
    servoInputBR = opMode.hardwareMap.analogInput.get("BREncoder");
    servoBR = opMode.hardwareMap.servo.get("BRServo");
    motorBR = opMode.hardwareMap.dcMotor.get("BRMotor");
    motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

    //BL
    servoInputBL = opMode.hardwareMap.analogInput.get("BLEncoder");
    servoBL = opMode.hardwareMap.servo.get("BLServo");
    motorBL = opMode.hardwareMap.dcMotor.get("BLMotor");
    motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

    double currentTime = Utils.getTimeSeconds();
    double lastTime = currentTime;
  }

  // distance to drive by encoder
  // turns on motor for a drive distance
  // dist in meters
public void driveDist(double dist, double mSpd){
    if(mSpd < .3)mSpd = 0.3;
    if(mSpd > 1.0)mSpd = 1.0;
    double encCt = wheelCircumferenceMeters * dist;
    motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int)(dist * encCt));
    motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorFR.setTargetPosition(motorFR.getCurrentPosition() + (int)(dist * encCt));
    motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int)(dist * encCt));
    motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motorBR.setTargetPosition(motorBR.getCurrentPosition() + (int)(dist * encCt));
    motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    forward(mSpd);
    while(opMode.opModeIsActive()&& (motorFL.isBusy() &&
            motorFR.isBusy()&&motorBL.isBusy()&&motorBR.isBusy())){
      alignWheels(wheelAngle);// compensate for drift
    }
    stopMotor();
}

  //odo.getPosX is forward on robot
  public void forward(double pwr){
    motorBL.setPower(pwr);
    motorBR.setPower(pwr);
    motorFL.setPower(pwr);
    motorFR.setPower(pwr);
  }
  public void stopMotor(){
    double mpwr = 0.0;
    motorBL.setPower(mpwr);
    motorBR.setPower(mpwr);
    motorFL.setPower(mpwr);
    motorFR.setPower(mpwr);
  }
  // align wheels has a angle always set where alignwheels is
  // constantly checking for the turn error and eventually compensating
  public void alignWheels(double wang){
    if(wang > 359.0) wang = 359.0;
    if(wang < 0.0) wang = 0.0;
    double servoCmd = wang/110.095;// finds the reference voltage of Servo Potentiometer

    // set direction of motors
    motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
    motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
    motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
    motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    //
    double FLstate = 0.0, FRstate = 0.0, BLstate = 0.0, BRstate = 0.0;
    // PID values for each; servo servo speed .5 is stop
    // PID value for reverse 0 - .4
    // PID value for forward .6 - 1.0
    double FLkp = 0.0;
    double FRkp = 0.0;
    double BLkp = 0.0;
    double BRkp = 0.0;
    double FLki = 0.0;
    double FRki = 0.0;
    double BLki = 0.0;
    double BRki = 0.0;
    double FLkd = 0.0;
    double FRkd = 0.0;
    double BLkd = 0.0;
    double BRkd = 0.0;
    while(opMode.opModeIsActive() && FLkd < 0.05 || FRkd < 0.05 || BLkd < 0.05 || BRkd < 0.05){
      // State of servos
      FLstate = servoInputFL.getVoltage();
      FRstate = servoInputFR.getVoltage();
      BLstate = servoInputBL.getVoltage();
      BRstate = servoInputBR.getVoltage();
      // reference = wang angle we want wheels
      // use PID to control convergence
      // kp = 0.5 + (reference - state)/reference = servo speed
      // if desired add ki and kd may control hunting
      // external sleep states can cause the robot to drift
      // start servo movement
      FLkd = servoCmd - FLstate;
      FRkd = servoCmd - FRstate;
      BLkd = servoCmd - BLstate;
      BRkd = servoCmd - BRstate;
      FLkp = 0.5 + FLkd/ servoCmd;
      FRkp = 0.5 + FRkd / servoCmd;
      BLkp = 0.5 + BLkd / servoCmd;
      BRkp = 0.5 + BRkd / servoCmd;
      if(FLkd < 0.05) servoFL.setPosition(.5); else servoFL.setPosition(FLkp);
      if(FRkd < 0.05) servoFR.setPosition(.5); else servoFR.setPosition(FRkp);
      if(BLkd < 0.05) servoBL.setPosition(.5); else servoBL.setPosition(BLkp);
      if(BRkd < 0.05) servoBR.setPosition(.5); else servoBR.setPosition(BRkp);
    }
    opMode.telemetry.addData("servoCmd", servoCmd );
    opMode.telemetry.addData("FL error", servoCmd - FLstate);
    opMode.telemetry.addData("FR error", servoCmd - FRstate);
    opMode.telemetry.addData("BL error",servoCmd - BLstate);
    opMode.telemetry.addData("BR error",servoCmd - BRstate);
    opMode.telemetry.addData("FL kp: ",0.5+(servoCmd-FLstate)/servoCmd);
    opMode.telemetry.addData("FR kp: ",0.5+(servoCmd-FRstate)/servoCmd);
    opMode.telemetry.addData("BL kp: ",0.5+(servoCmd-BLstate)/servoCmd);
    opMode.telemetry.addData("BR kp: ",0.5+(servoCmd-BRstate)/servoCmd);
    telem();
    opMode.sleep(5000);
  }

  public void telem(){
    opMode.telemetry.addData("ServoFR voltage: ",servoInputFR.getVoltage());
    opMode.telemetry.addData("servoFL voltage: ",servoInputFL.getVoltage());
    opMode.telemetry.addData("ServoBR voltage: ",servoInputBR.getVoltage());
    opMode.telemetry.addData("servoBL voltage: ",servoInputBL.getVoltage());
    opMode.telemetry.update();
  }

}
