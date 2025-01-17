package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import java.util.ArrayList;

public class ArmControl extends LinearOpMode {
  public int linearRailIdx = 0;
  public float linearRailRotation = 0;
  private List<Integer> sigma = new ArrayList<>();
  public int clawIdx = 1;
  public float clawRotation = 0;
  public int armIdx = 2;
  public float armRotation = 0;
  
  // HOW TO REMOVE LIMITS FOR SERVO ROTATIONS: Set maxRot to 9999 or set minRot to -9999

  public float[] minRot = List.of(0, 0);
  public float[] maxRot = List.of(4, 4);
  
  private Gamepad gamepad1 = new Gamepad(); // Still needs some work...
  public List<Servo> servoIdentity = List.of(hardwareMap.get(Servo.class, "N/A"), hardwareMap.get(Servo.class, "N/A"), hardwareMap.get(Servo.class, "N/A"));
  public void runOpMode() throws InterruptedException {


    servoIdentity.get(linearRailIdx).setDirection(Servo.Direction.FORWARD);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait to press START on controller
    waitForStart();

    while (opModeIsActive()) {
      SetServoPosition(linearRailIdx, linearRailRotation);
      ConfineServoBoundaries();
      
      telemetry.addData("Servo Position", linearRailPosition);
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
    servoIdentity.get(servoIdx).setPosition(position);
  }
}
