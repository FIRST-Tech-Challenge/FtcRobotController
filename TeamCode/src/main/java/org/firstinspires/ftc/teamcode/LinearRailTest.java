package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LinearRailTest extends LinearOpMode {
  // SERVO ORDER: Linear Rail, Claw
  public List<Servo> servoIdentity = List.of(hardwareMap.get(Servo.class, "N/A"), hardwareMap.get(Servo.class, "N/A")); 
  public List<Int> servoRotations = List.of(0, 0)
  public List<float> minRot = List.of(0, 0)
  public List<float> maxRot = List.of(4, 4)
  
  private Gamepad gamepad1 = new Gamepad(); // Still needs some work...
  
  public void runOpMode() throws InterruptedException {
    servoIdentity.get(0).setDirection(Servo.Direction.FORWARD);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait to press START on controller
    waitForStart();

    while (opModeIsActive()) {
      SetServoPosition(0, servoRotations.get(0));
      ConfineServoBoundaries();
      
      telemetry.addData("Servo Position", servoIdentity.get(0).getPosition());
      telemetry.addData("Status", "Running");
      telemetry.update();
    }
  }

  public static void ConfineServoBoundaries() {
    for (int i=0; i<servoRotations.Length; i++) {
      if (servoRotations.get(i) <= minRot.get(i)) {
        servoRotations.get(i) = minRot.get(i);
      }

      if (servoRotations.get(i) >= maxRot.get(i)) {
        servoRotations.get(i) = maxRot.get(i);
      }
    }
  }

  public static void SetServoRotation(int servoIdx, float position) {
    servoIdentity.get(servoIdx).setPosition(position);
  }
}
