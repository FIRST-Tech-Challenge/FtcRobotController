package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import java.util.ArrayList;

public class ArmControl extends LinearOpMode {
  public int linearRailIdx = 0;
  public int clawIdx = 1;
  public int armIdx = 2;

  public float[] servoRotations = List.of(0, 0, 0;
  
  // HOW TO REMOVE LIMITS FOR SERVO ROTATIONS: Set maxRot to 9999 or set minRot to -9999

  public float[] minRot = List.of(0, 0);
  public float[] maxRot = List.of(4, 4);
  
  private Gamepad gamepad = new Gamepad(); // Still needs some work...
  public List<Servo> servoIdentity = List.of(hardwareMap.get(Servo.class, "N/A"), hardwareMap.get(Servo.class, "N/A"), hardwareMap.get(Servo.class, "N/A"));

  // NEED TO ADJUST CLOSE AND OPEN CLAW POSITIONS
  public float closeClawPosition = 0;
  public float openClawPosition = 0;
  
  public void runOpMode() throws InterruptedException {
    telemetry.addData("Status", "Initialized, Waiting For Controller Input");
    telemetry.update();

    // Wait to press START on controller
    waitForStart();

    while (opModeIsActive()) {
      if (gamepad.y) {
        SetServoPosition(linearRailIdx, servoRotations.get(linearRailIdx) + 1);
      }

      if (gamepad.a) {
        SetServoPosition(linearRailIdx, servoRotations.get(linearRailIdx) - 1);
      }

      if (gamepad.b) {
        SetServoPosition(clawIdx, closeClawPosition);
      } else {
        SetServoPosition(clawIdx, openClawPosition);
      }
      
      ConfineServoBoundaries();
      
      telemetry.addData("Servo Position: LinearRail", servoRotations[linearRailIdx]);
      telemetry.addData("Servo Position: Claw", servoRotations[clawIdx]);
      telemetry.addData("Servo Position: Arm", servoRotations[armIdx]);
      telemetry.addData("Status", "Running");
      telemetry.update();
    }
  }

  public void ConfineServoBoundaries() {
    for (int i=0; i<servoRotations.Length; i++) {
      if (servoRotations.get(i) != 9999) {
        if (servoRotations.get(i) <= minRot.get(i)) {
          servoRotations.get(i) = minRot.get(i);
        }
      }
      
      if (servoRotations.get(i) != -9999) {
        if (servoRotations.get(i) >= maxRot.get(i)) {
          servoRotations.get(i) = maxRot.get(i);
        } 
      }
    }
  }

  public void SetServoRotation(int servoIdx, float position) {
    servoIdentity.set(servoIdx).setPosition(position);
  }
