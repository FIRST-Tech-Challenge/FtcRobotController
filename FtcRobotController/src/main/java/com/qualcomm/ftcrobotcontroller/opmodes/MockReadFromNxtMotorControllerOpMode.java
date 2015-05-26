package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This op mode uses the NXT DC Motor Controller to drive the motor based on the encoder values.
 *
 * These method calls do not exists currently. It will take 5-7 days to implement them.
 *
 * The USB DC Motor Controller will have a much simpler API. e.g. value = motor.getEncoderValue();
 */
public class MockReadFromNxtMotorControllerOpMode extends OpMode {

  DcMotor motor = null;

  boolean waitingOnCallback = false;
  int motorEncoderValue = 0;
  int runNumber = 0;

  @Override
  public void start() {
    motor = hardwareMap.dcMotor.get("arm");
  }

  @Override
  public void loop() {

    // we need to check if we are waiting on a callback. if we are waiting on a
    // callback we should not perform any action on the motor
    if (waitingOnCallback == false) {

      // every ten runs we will check the encoders (not counting runs where we are waiting on a
      // callback)
      if (runNumber % 10 == 0) {

        // this is the tenth run, so we will check the encoder

        // mark that we are waiting on a callback
        waitingOnCallback = true;

        // call to get the encoder value, passing in a callback object
        motor.getEncoders(new DcMotor.MotorCallback() {
          @Override
          public void encoder(int value) {

            // At some point in the future, probably 100-200ms from now, this code will run
            // It is important to note that this code will NOT run in the same thread as the run
            // loop.

            // we should cache the encoder value for use when the loop method is called
            motorEncoderValue = value;

            // we are no longer waiting on a callback
            waitingOnCallback = false;
          }
        });
      } else {

        // this is not the tenth run, so we will advance the motor as long as the encoder value is
        // below our threshold
        if (motorEncoderValue < 10000) {
          motor.setPower(0.5);
        }
      }
      runNumber += 1;
    }
  }

  @Override
  public void stop() {
    // no action needed
  }
}
