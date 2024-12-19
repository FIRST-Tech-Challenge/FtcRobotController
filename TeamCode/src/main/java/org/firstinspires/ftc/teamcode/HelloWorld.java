package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HelloWorld extends OpMode {

  @Override
  public void init() {
      telemetry.addData("Hello", "Jasper");
      telemetry.addData("Hello", "Isaiah");
  }

      @Override
              public void loop() {
                telemetry.addData("Bye", "Guys");
      }
  }

