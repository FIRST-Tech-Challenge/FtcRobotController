/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.android.OpModes;

import android.os.SystemClock;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.IDataArrivalSubscriber;

import java.text.DecimalFormat;

/**
 * navX-Micro Collision Detection Op
 *
 * This is a demo program showing the use of the navX-Micro to implement
 * a collision detection feature, which can be used to detect events
 * while driving a robot, such as bumping into a wall or being hit
 * by another robot.
 *
 * The basic principle used within the Collision Detection example
 * is the calculation of Jerk (which is defined as the change in
 * acceleration).  In the sample code shown below, both the X axis and
 * the Y axis jerk are calculated, and if either exceeds a threshold,
 * then a collision has occurred.
 *
 * The 'collision threshold' used in these samples will likely need to
 * be tuned for your robot, since the amount of jerk which constitutes
 * a collision will be dependent upon the robot mass and expected
 * maximum velocity of either the robot, or any object which may strike
 * the robot.
 *
 * Note that this example uses the "callback" mechanism to be informed
 * precisely when new data is received from the navX-Micro.
 */
@TeleOp(name = "Concept: navX Collision Detection", group = "Concept")
@Disabled
public class ConceptNavXCollisionDetectionOp extends OpMode implements IDataArrivalSubscriber {

  /* Tune this threshold to adjust the sensitivy of the */
  /* Collision detection.                               */
  private final double COLLISION_THRESHOLD_DELTA_G = 0.5;

  double last_world_linear_accel_x;
  double last_world_linear_accel_y;
  private ElapsedTime runtime = new ElapsedTime();
  private AHRS navx_device;
  private boolean collision_state;

  private final String COLLISION = "Collision";
  private final String NO_COLLISION = "--------";

  private long last_system_timestamp = 0;
  private long last_sensor_timestamp = 0;

  private long sensor_timestamp_delta = 0;
  private long system_timestamp_delta = 0;

  @Override
  public void init() {
    navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
            AHRS.DeviceDataType.kProcessedData);
    last_world_linear_accel_x = 0.0;
    last_world_linear_accel_y = 0.0;
    setCollisionState(false);
  }

@Override
  public void start() {
    navx_device.registerCallback(this);
}

  @Override
  public void stop() {
    navx_device.close();
  }
  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    telemetry.addData("navX Op Init Loop", runtime.toString());
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {

      boolean connected = navx_device.isConnected();
      telemetry.addData("1 navX-Device", connected ?
              "Connected" : "Disconnected" );
      String gyrocal, motion;
      DecimalFormat df = new DecimalFormat("#.##");

      if ( connected ) {
          gyrocal = (navx_device.isCalibrating() ?
                  "CALIBRATING" : "Calibration Complete");
          motion = (navx_device.isMoving() ? "Moving" : "Not Moving");
          if ( navx_device.isRotating() ) {
              motion += ", Rotating";
          }
      } else {
          gyrocal =
            motion = "-------";
      }
      telemetry.addData("2 GyroAccel", gyrocal );
      telemetry.addData("3 Motion", motion);
      telemetry.addData("4 Collision", getCollisionString());
      telemetry.addData("5 Timing", Long.toString(sensor_timestamp_delta) + ", " +
                                    Long.toString(system_timestamp_delta) );
      telemetry.addData("6 Events", Double.toString(navx_device.getUpdateCount()));
  }

  private String getCollisionString() {
      return (this.collision_state ? COLLISION : NO_COLLISION);
  }

    private void setCollisionState( boolean newValue ) {
      this.collision_state = newValue;
  }

  /* This callback is invoked by the AHRS class whenever new data is
     received from the sensor.  Note that this callback is occurring
     within the context of the AHRS class IO thread, and it may
     interrupt the thread running this opMode.  Therefore, it is
     very important to use thread synchronization techniques when
     communicating between this callback and the rest of the
     code in this opMode.

     The difference between the current linear acceleration data in
     the X and Y axes and that in the last sample is compared.  If
     the absolute value of that difference is greater than the
     "Collision Detection Threshold", a collision event is declared.
  */

    @Override
    public void timestampedDataReceived(long curr_system_timestamp, long curr_sensor_timestamp, Object o) {
        boolean collisionDetected = false;

        sensor_timestamp_delta = curr_sensor_timestamp - last_sensor_timestamp;
        system_timestamp_delta = curr_system_timestamp - last_system_timestamp;
        double curr_world_linear_accel_x = navx_device.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = navx_device.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkY) > COLLISION_THRESHOLD_DELTA_G) ) {
            collisionDetected = true;
        }

        setCollisionState( collisionDetected );
    }

    @Override
    public void untimestampedDataReceived(long l, Object o) {

    }

    @Override
    public void yawReset() {
    }
}
