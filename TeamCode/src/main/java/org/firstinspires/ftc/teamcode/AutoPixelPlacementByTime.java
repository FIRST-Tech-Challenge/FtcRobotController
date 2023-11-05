/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoPixelPlacementByTime.Stages.*;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
@TeleOp(name = "Hornet Pixel Placement", group = "Sensor")
public class AutoPixelPlacementByTime extends LinearOpMode {

    private DistanceSensor sensorDistance;
    private RobotHardware robot = new RobotHardware(this);
    static final double FORWARD_SPEED = 0.2;
    static final double TARGET_DISTANCE_INCHES = 2590;
    private ElapsedTime runtime = new ElapsedTime();

    public enum Stages {
        ROBOT_READY_TO_FORWARD_MOVE,
        ELBOW_READY_TO_MOVE_DOWW,
        ELBOW_STOPPED,
        GRABBER_READY_TO_OPEN
    }

    private Stages CurrentStatus = ROBOT_READY_TO_FORWARD_MOVE;

    static final int forwardPeriod = 5500; //in milli seconds
    @Override
    public void runOpMode() {
        robot.init();
        runtime.reset();
        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive() ) {

            switch(CurrentStatus){
                case ROBOT_READY_TO_FORWARD_MOVE:
                    moveRobotToPosition();
                    break;
                case ELBOW_READY_TO_MOVE_DOWW:
                    moveElbowToPosition();
                    break;
                case GRABBER_READY_TO_OPEN:

                    break;
                default:
                    break;
            }



            telemetry.update();
        }
    }

    private void moveElbowToPosition(){
        //
        //move the elbow till it gets to a specific position
        if (robot.getGrabberPoistion() < 0.80) {
            telemetry.addData("Reached Target Distance, Drop pixel", "");
            robot.moveElbow(false);
            //robot.moveElbowToPosition(0.80); //Move Elbow all the way down
            //robot.moveGrabber(true); //release grabber to drop pixel
            telemetry.addData("Status", "In else condition");
            sleep(10);
        }
        else{
            CurrentStatus = ELBOW_STOPPED;
        }
    }
    private void moveRobotToPosition(){
        if ( runtime.milliseconds() <forwardPeriod) {
            robot.driveRobot(FORWARD_SPEED, 0);
            // generic DistanceSensor methods.
            telemetry.addData("run time ", runtime.milliseconds());
            telemetry.addData("deviceName", sensorDistance.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
        }
        else{
            robot.driveRobot(0, 0);
            CurrentStatus = ELBOW_READY_TO_MOVE_DOWW;
        }
    }
    private boolean hasReachedTargetDistance(DistanceSensor sensorDistance){
        if (sensorDistance.getDistance(DistanceUnit.INCH) >= TARGET_DISTANCE_INCHES)
            return true;
        return false;
    }
}
