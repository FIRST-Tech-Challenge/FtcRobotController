/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 *  This particular OpMode illustrates testing the spin direction of a holonomic robot
 *  Notes:
 *  - A Mecanum drive must display an X roller-pattern in plan view (top/above)
 *  - Set the correct rotation direction for each motor
 *
 *  Button  Alternate    Wheel        Port    Setting
 *    X       Square      Left Front  0       FORWARD
 *    A       Cross       Left Back   1       FORWARD
 *    Y       Triangle    Right Front 2       REVERSE
 *    B       Circle      Right Back  3       REVERSE
 *
 * @link https://youtu.be/DN7KW_fiVR4
 *  */
@TeleOp(name = "Mecanum: Wheel Spin", group = "Test")
//@Disabled

public class MecanumWheelSpin extends LinearOpMode {

  // Declare OpMode members for each of the 4 motors.
  private final ElapsedTime runtime = new ElapsedTime();
                                       // variables for motors
  String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
  private final DcMotorEx[] motor = {null, null, null, null}; // reservations only, please
  private final double[] motorPower = {0.0, 0.0, 0.0, 0.0};

  Orientation orientation;

  @Override
  public void runOpMode() {

    // Initialize the hardware variables. Note that the strings used here must correspond
    // to the names assigned during the robot configuration step on the DS or RC devices
    double SPIN_POWER = 0.3;

    for (int i = 0; i < motor.length; i++) {
      motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
                                        /* motor stops and then brakes
                                         * actively resisting any external force
                                         * which attempts to turn the motor */
      motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                                        // run at any velocity with specified power level
      motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ########################################################################################
    // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
    // ########################################################################################
    // Most robots need the motors on one side to be reversed to drive forward.
    // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
    // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
    // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
    // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
    // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
    // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
    /*
     *  Button  Alternate   Wheel       Port    Setting
     *  X       Square      Left Front  0       FORWARD
     *  A       Cross       Left Back   1       FORWARD
     *  Y       Triangle    Right Front 2       REVERSE
     *  B       Circle      Right Back  3       REVERSE
     */
    motor[0].setDirection(DcMotorEx.Direction.REVERSE);
    motor[1].setDirection(DcMotorEx.Direction.REVERSE);
    motor[2].setDirection(DcMotorEx.Direction.FORWARD);
    motor[3].setDirection(DcMotorEx.Direction.FORWARD);

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Status", "Press Start to continue");
    telemetry.update();

    waitForStart();

    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      /*
       * This is test code:
       *
       * Uncomment the following code to test your motor directions.
       * Each button should make the corresponding motor run FORWARD.
       * 1) First get all the motors to take to correct positions on the robot
       *    by adjusting your Robot Configuration if necessary.
       * 2) Then make sure they run in the correct direction by modifying the
       *    the setDirection() calls above.
      // Once the correct motors move in the correct direction re-comment this code.
      */
      motorPower[0]  = gamepad1.x ? SPIN_POWER : 0.0;  // X gamepad
      motorPower[1]  = gamepad1.a ? SPIN_POWER : 0.0;  // A gamepad
      motorPower[2]  = gamepad1.y ? SPIN_POWER : 0.0;  // Y gamepad
      motorPower[3]  = gamepad1.b ? SPIN_POWER : 0.0;  // B gamepad

      for (int i = 0; i < motor.length; i++) {
        motor[i].setPower(motorPower[i]);// Send calculated power to wheels
      }

      telemetry.addData("Status", "Run Time: " + runtime.seconds()); // Show the elapsed game time and wheel power
      telemetry.addData("Gamepad", "Press a button from list below");
      telemetry.addData("Front  ", "%s, %s", "Left: X Square ", "Right: Y Triangle");
      telemetry.addData("Back   ", "%s, %s", "Left: A Cross  ", "Right: B Circle");
      telemetry.update();
    }
  }

}