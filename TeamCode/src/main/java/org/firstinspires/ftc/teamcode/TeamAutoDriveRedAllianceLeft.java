///* Copyright (c) 2023 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//
//@Autonomous(name="RED", group = "Concept")
////@Disabled
//public class TeamAutoDriveRedAllianceLeft extends LinearOpMode
//{
//    private TeamAutoDrive tad ; //= new TeamAutoDrive(hardwareMap, telemetry);
//
//    //static final double     DRIVE_SPEED             = 1;
//    //static final double     TURN_SPEED              = 0.5;
//    //private float turn_distance = 24;
////    private float forward_distance = 3;
////    private float reverse_distance = 3;
//    //private float team_object_distance = 25;
//    private static final String TFOD_MODEL_FILE = "TeamPropAbs0RED.tflite";//"/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
//
//
//    @Override public void runOpMode()
//    {
//        tad = new TeamAutoDrive(hardwareMap, telemetry, gamepad1, TFOD_MODEL_FILE);
//
//        // Wait for driver to press start
//        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            int obj_location = -1;
//            try {
//                obj_location = tad.teamObjectDetectionTfod();
//
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//            //drive to team object
//            obj_location = driveToTeamObject(obj_location);
//
//            //drive to April Tag
//            //for RED side add 3 to object location to match to april tag number
//            int april_tag_number = obj_location + 3;
//            tad.driveToTeamAprilTag(april_tag_number);
//            tad.dropPixel();
//            tad.parkRobot(april_tag_number);
//            break;
//        }
//    }
//
//    private int driveToTeamObject(int obj_location){
//        int team_object_position = obj_location; // 2 - Center, 1 left and 3 right
//        telemetry.addData("Auto - move to team object","Drive %5.2f inches ", tad.team_object_distance);
//        telemetry.update();
//
//        // if team object not found then try again, if found move forward
//        if (obj_location == -1) {
//            team_object_position = tad.tryAgainTeamObjectDetection();
//        } else {
//            tad.driveRobot(tad.DRIVE_SPEED,  tad.team_object_distance,  tad.team_object_distance, 4.0);  // S1: Forward 27 Inches with 5 Sec timeout
//        }
//        // Decide what to do based on position
//        // if center then put pixel next to team object, go back 2 inch and turn left
//        // if left then turn left, move forward 2 inches, put pixel next to team object, move back 2 inches, move left 8 inches
//        // if right then turn right, move forward 2 inches, put pixel next to team object, move back 2 inches, turn 180 degrees
//        if (team_object_position == 2) {
//            // if team object is in center
//            //move forward, push tray and move back and turn towards April Tag
//            // tad.driveRobot(DRIVE_SPEED, forward_distance, forward_distance, 3.0);
//            tad.pushTrayPixel(1500,tad.forward_distance, tad.reverse_distance);
//            // turn left towards the board
//            tad.driveRobot(tad.TURN_SPEED,   tad.turn_distance, -tad.turn_distance, 4.0);
//            tad.driveRobot(tad.DRIVE_SPEED, tad.backdrop_cross_distance, tad.backdrop_cross_distance, 4.0);
//            tad.moveParallelToRightD(tad.DRIVE_SPEED, 2, 2.0);
//        } else if (team_object_position == 3){
//            // if team object position is right
//            // turn left towards the board
////          tad.moveParallelToLeft(400);
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, tad.move_away_distance, 2.0);
//            //turn right
//            tad.driveRobot(tad.TURN_SPEED, tad.turn_distance, -tad.turn_distance, 3.0);
//            // move more left to make sure pixel is dropped on line
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 9, 2.0);
//            //move forward and drop the pixel
//            //since moved little to left add to forward distance and to avoid hitting the back go less reverse
//            tad.pushTrayPixel(1500, (float) (tad.forward_distance+tad.move_away_adjustment),tad.reverse_distance-(float)3.0);
//            //tad.moveParallelToRight(1200);
//            //try to cross the team element and pixel
//            tad.moveParallelToRightD(tad.DRIVE_SPEED, 18, 2.0);
//            tad.driveRobot(tad.DRIVE_SPEED, tad.backdrop_cross_distance, tad.backdrop_cross_distance, 3.0);
////            tad.moveParallelToLeft(600);
//            //align to April Tag
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 5, 2.0);
//        } else if (team_object_position == 1){
//            // if team object position is left
//            // turn right towards the team object
//            // move little left before turning right
//            tad.moveParallelToRightD(tad.DRIVE_SPEED, tad.move_away_distance, 2.0);
//            //tad.moveParallelToRight(400);
//            tad.driveRobot(tad.TURN_SPEED, -tad.turn_distance, tad.turn_distance, 3.0);
//            tad.pushTrayPixel(1500, (float) (tad.forward_distance+tad.move_away_adjustment), tad.reverse_distance+2);
//            //turn 180 degrees
//            tad.driveRobot(tad.TURN_SPEED,   (tad.turn_distance*2)-0.5, -(tad.turn_distance*2)-0.5, 6.0);
//            //drive towards April Tag
//            tad.driveRobot(tad.DRIVE_SPEED, tad.backdrop_cross_distance-5, tad.backdrop_cross_distance-5, 3.0);
//            //align to April Tag
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 6, 2.0);
//        } else {
//            telemetry.addData("Not able  to find object"," object position %d ", team_object_position);
//            telemetry.update();
//            team_object_position = 2;
//            sleep(1000);
//        }
//        return team_object_position;
//    }
//}