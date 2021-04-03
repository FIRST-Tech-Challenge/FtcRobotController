/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Skystone", group ="Disabled")

public class RedSkystone extends LinearOpMode implements MecanumDrive.TickCallback {

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private AtlasRobot robot = new AtlasRobot();
    private SkystoneObjectDetector tf;

    enum RedScan {LEFT, MIDDLE, RIGHT}
    int updates;

    @Override
    public void runOpMode() {
        tf = new SkystoneObjectDetector();
        tf.init(hardwareMap, telemetry, this);

        while (!isStarted() ) {
            telemetry.addData("path", tf.choosePath());
            telemetry.update();
            sleep(100);
        }

        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);
        telemetry.addData("Status", "Initialized");
        mecanumDrive.setupTickCallback(this);

        waitForStart();
        robot.inRamp.setPosition(1);
        int path = 0;
        int loops = 0;
        for (int i = 1; i<60; i++) {
            path = tf.choosePath();
            loops=i;
            if (path > 0) {
                break;
            }
            telemetry.addData("loop", i);
            telemetry.update();
            sleep(50);
        }
        if (path==0) {
            path = 1;
        }

        if (path == 1) {
            redScan(RedScan.RIGHT);
        }else if (path == 2) {
            redScan(RedScan.MIDDLE);
        }else if (path == 3) {
            redScan(RedScan.LEFT);
        }else {}
    }

    void redScan(RedScan output) {
        switch (output) {
            case RIGHT:
                mecanumDrive.backward(22, .8);
                //Pick up first skystone
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN, true);
                mecanumDrive.arcMove( 8, -90, .5, MecanumDrive.MoveDirection.LEFT, false, true);
                mecanumDrive.leftStrafe(14,1);
                //sleep(400);
                //mecanumDrive.turnTo(-91,.05);
                //crossing under bridger for the first time
                mecanumDrive.backward(84,.8);
                mecanumDrive.turnTo(180,.3);
                mecanumDrive.forward(9,.3);
                //move foundation
                moveFoundation();
                mecanumDrive.leftStrafe(12, .8);
                sleep(200);
                mecanumDrive.turnTo(92, .05);
                mecanumDrive.backward(75,.8);
                mecanumDrive.turnTo(45,.3);
                //second sky-stone
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN,true);
                mecanumDrive.backward(20,.3);
                mecanumDrive.forward(20,.3);
                mecanumDrive.turnTo(90,.3);
                mecanumDrive.forward(89,1);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
                sleep(1000);
                //mecanumDrive.turnTo(90,.1);
                //park
                mecanumDrive.backward(72-27,.8);

                break;
            case MIDDLE:
                mecanumDrive.freeWheel(-0.8,-0.1,-0.8,-0.1,5);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN,true);
                mecanumDrive.diagonal(30,-1, MecanumDrive.MoveDirection.LEFT, false);
                mecanumDrive.freeWheel(-0.5,-0.5,-0.5,-0.5,5);
                mecanumDrive.forward(8,.5,true);
                mecanumDrive.turnTo(88,.3);
                //crossing under bridger for the first time
                mecanumDrive.forward(87,.8,true);
                mecanumDrive.turnTo(180,.3);
                mecanumDrive.forward(9,.3);
                //move foundation
                moveFoundation();
                mecanumDrive.leftStrafe(10, .8);
                sleep(200);
                mecanumDrive.turnTo(92, .05);
                mecanumDrive.backward(80,.8);
                mecanumDrive.turnTo(45,.3);
                //Second skystone
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN,true);
                mecanumDrive.backward(20,.3);
                mecanumDrive.forward(21,.3);
                mecanumDrive.turnTo(91,.3);
                mecanumDrive.forward(89,1);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
                sleep(1000);
                //park
                mecanumDrive.backward(72-27,.8);
                break;
            case LEFT:
                mecanumDrive.freeWheel(-0.8,-0.1,-0.8,-0.1,8);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN,true);
                mecanumDrive.diagonal(26,-1, MecanumDrive.MoveDirection.LEFT, true);
                mecanumDrive.freeWheel(-0.5,-0.5,-0.5,-0.5,7);
                mecanumDrive.forward(12,.3,true);
                mecanumDrive.turnTo(88,0.3);
                mecanumDrive.turnTo(90,0.3);
                mecanumDrive.forward(95,.8,true);
                mecanumDrive.turnTo(180,.3);
                mecanumDrive.forward(9,.3);
                //move foundation
                moveFoundation();
                mecanumDrive.leftStrafe(10, .8);
                sleep(200);
                mecanumDrive.turnTo(90, .3);
                mecanumDrive.backward(1,.1);
                mecanumDrive.turnTo(91,.05);
                mecanumDrive.backward(90,.8);
                mecanumDrive.leftStrafe(11,.5);
                //Second skystone
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN,true);
                mecanumDrive.backward(8,.3);
                mecanumDrive.forward(8,.3);
                mecanumDrive.rightStrafe(11,.5);
                mecanumDrive.turnTo(90,.3);
                mecanumDrive.forward(1,.1);
                mecanumDrive.turnTo(90,0.05);
                mecanumDrive.forward(103,1);
                robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
                sleep(1000);
                //park
                mecanumDrive.turnTo(90,.3);
                mecanumDrive.turnTo(91,.05);
                mecanumDrive.backward(72-24,.8);
                break;
        }

    }
    void moveFoundation() {
        robot.foundationMover(false);
        sleep(1000);
        robot.setManipulator(AtlasRobot.ManipulatorDirection.IN);
        mecanumDrive.freeWheel(-0.1, -1, -0.1, -1, 24);
        mecanumDrive.freeWheel(0.2, -1, 0.2, -1, 18);

        robot.setManipulator(AtlasRobot.ManipulatorDirection.STOP);
        robot.foundationMover(true);
        //mecanumDrive.backward(6,1);
        //mecanumDrive.leftStrafe(12, .8);
        //sleep(200);
        //mecanumDrive.turnTo(94, .05);
    }
    public void tickCallback() {
        updates++;
        if ( robot.switchPressed() ) {
            robot.manipulatorAutostop();
        }
        telemetry.addData("I'm running callback code rightnow!", updates);
        telemetry.update();
    }
}