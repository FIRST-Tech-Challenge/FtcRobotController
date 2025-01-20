package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

public class AutoSwerve {

  AnalogInput servoInputFL,servoInputFR,servoInputBR,servoInputBL;
  Servo servoFL,servoFR,servoBR,servoBL;
  DcMotor motorFL,motorFR,motorBR,motorBL;
  LinearOpMode opMode;
  GoBildaPinpointDriver odo;
  public AutoSwerve(LinearOpMode opMode,GoBildaPinpointDriver odo){
    this.opMode = opMode;
    this.odo = odo;
    //FL
    servoInputFL = opMode.hardwareMap.analogInput.get("FLEncoder");
    servoFL = opMode.hardwareMap.servo.get("FLServo");
    motorFL = opMode.hardwareMap.dcMotor.get("FLMotor");

    //FR
    servoInputFR = opMode.hardwareMap.analogInput.get("FREncoder");
    servoFR = opMode.hardwareMap.servo.get("FRServo");
    motorFR = opMode.hardwareMap.dcMotor.get("FRMotor");
    motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

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
  }

  //odo.getPosX is forward on robot
  public void forward(double distance){
    motorBL.setPower(distance);
    motorBR.setPower(distance);
    motorFL.setPower(distance);
    motorFR.setPower(distance);
  }

  public void alignWheels(){
    while(!(3.2<=servoInputFL.getVoltage() && servoInputFL.getVoltage()<=3.3)) {
      servoFL.setPosition(0.35);
      telem();
    }
    servoFL.setPosition(0.5);
    while(!(1.5<=servoInputFR.getVoltage() && servoInputFR.getVoltage()<=1.65)) {
      servoFR.setPosition(0.35);
      telem();
    }
    servoFR.setPosition(0.5);
    while(!(1.5<=servoInputBL.getVoltage() && servoInputBL.getVoltage()<=1.65)) {
      servoBL.setPosition(0.35);
      telem();
    }
    servoBL.setPosition(0.5);
    while(!(3.<=servoInputBR.getVoltage() && servoInputBR.getVoltage()<=3.35)) {
      servoBR.setPosition(0.35);
      telem();
    }
    servoBR.setPosition(0.5);
  }

  public void telem(){
    opMode.telemetry.addData("ServoFR voltage: ",servoInputFR.getVoltage());
    opMode.telemetry.addData("servoFL voltage: ",servoInputFL.getVoltage());
    opMode.telemetry.addData("ServoBR voltage: ",servoInputBR.getVoltage());
    opMode.telemetry.addData("servoBL voltage: ",servoInputBL.getVoltage());
    opMode.telemetry.update();
  }
}
