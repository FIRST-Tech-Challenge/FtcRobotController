package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearRailTest extends LinearOpMode {
  private Servo testServo = hardwareMap.get(Servo.class, "N/A");
  public int servoRotation = 0;
  public float minRot = 0;
  public float maxRot = 4;
  
  public void runOpMode() throws InterruptedException {
    testServo = hardwareMap.get(Servo.class, "N/A");
    testServo.setPower(100);
    testServo.setDirection()

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait to press START on controller
    waitForStart();

    while (opModeIsActive()) {
      // check to see if we need to move the servo.
      if(gamepad1.y) {
          servoRotation += 0.1;
      } 
      
      if (gamepad1.x) {
          servoRotation -= 0.1;
      }

      // Confine Boundaries
      if (servoRotation <= 0) {
        servoRotation = minRot;
      }

      if (servoRotation >= 4) {
        servoRotation = maxRot;
      }

      testServo.setPosition(servoRotation);
      
      telemetry.addData("Servo Position", testServo.getPosition());
      telemetry.addData("Status", "Running");
      telemetry.update();
    }
}
