package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LinearRailTest extends LinearOpMode {
  // SERVO ORDER: Linear Rail, Claw
  public String[] servoIdentity = [hardwareMap.get(Servo.class, "N/A"), hardwareMap.get(Servo.class, "N/A");] 
  public float[] servoRotations = [0, 0]
  public float[] minRot = [0, 0]
  public float[] maxRot = [0, 0]
  
  private Gamepad gamepad1 = new Gamepad(); // Still needs some work...
  
  public void runOpMode() throws InterruptedException {
    linearRail = hardwareMap.get(Servo.class, servoIdentity[0]);
    claw = hardwareMap.get(Servo.class, servoIdentity[1]);
    linearRail.setDirection(Servo.Direction.FORWARD);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait to press START on controller
    waitForStart();

    while (opModeIsActive()) {
      if(gamepad1.y) {
          servoRotations[0] += 0.1;
      } 
      
      if (gamepad1.x) {
          servoRotations[0] -= 0.1;
      }

      if(gamepad1.a) {
          servoRotations[1] += 0.1;
      } 
      
      if (gamepad1.b) {
          servoRotations[1] -= 0.1;
      }

      ConfineServoBoundaries();
      
      SetServoPosition(0, servoRotations[0]);
      
      telemetry.addData("Servo Position", linearRail.getPosition());
      telemetry.addData("Status", "Running");
      telemetry.update();
    }
  }

  public static void ConfineServoBoundaries() {
    idx = 0;
    for servo in servoIdentity {
      if (servoRotations[idx] <= minRot[idx]) {
        servoRotations[idx] = minRot[idx];
      }

      if (servoRotations[idx] >= maxRot[idx]) {
        servoRotations[idx] = maxRot[idx];
      }

      idx++;
    }
  }

  public static void SetServoRotation(int servoIdx, float position) {
    servoIdentity[servoIdx].setPosition(position);
  }
}
