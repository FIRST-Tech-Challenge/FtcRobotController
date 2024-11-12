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
//
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//
//@Autonomous(name="FRONT BLUE", group = "Concept")
////@Disabled
//public class TeamAutoDriveBlueAllianceFront extends LinearOpMode
//{
//    private TeamAutoDrive tad ; //= new TeamAutoDrive(hardwareMap, telemetry);;
//
////    static final double     DRIVE_SPEED             = 1;
////    static final double     TURN_SPEED              = 0.6;
////    private float turn_distance = 27;
////    private float forward_distance = 3;
////    private float reverse_distance = 5;
//    private static final String TFOD_MODEL_FILE = "TeamPropAbs0.tflite";//"/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
//    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
//
//    @Override public void runOpMode()
//    {
//
//        tad = new TeamAutoDrive(hardwareMap, telemetry, gamepad1, TFOD_MODEL_FILE);
//        //Robot robot = new Robot(hardwareMap, telemetry);
//
//        // Wait for driver to press start
//        telemetry.addData("Camera preview o n/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            //targetFound = false;
//            desiredTag  = null;
//            int obj_location = -1;
//            try {
//                obj_location = tad.teamObjectDetectionTfod();
//
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//            //drive to team object
//            obj_location = driveToTeamObject(obj_location);
//            //drive to April Tag
//            tad.driveToTeamAprilTag(obj_location);
//            tad.dropPixel();
//            tad.parkRobot(obj_location);
//            break;
//        }
//    }
//
//    private int driveToTeamObject(int obj_location){
//        int team_object_position = obj_location; // 2 - Center, 1 left and 3 right
//
//        telemetry.addData("Auto - move to team object","Drive %5.2f inches ", tad.team_object_distance);
//        telemetry.update();
//        // if object not found at start then try again else move forward
//        if (obj_location == -1) {
//            team_object_position = tad.tryAgainTeamObjectDetection();
//        } else {
//            tad.driveRobot(tad.DRIVE_SPEED,  tad.team_object_distance,  tad.team_object_distance, 4.0);  // S1: Forward 24 Inches with 5 Sec timeout
//        }
//        // Decide what to do based on position
//        // if center then put pixel next to team object, go back 2 inch and turn left
//        // if left then turn left, move forward 2 inches, put pixel next to team object, move back 2 inches, move left 8 inches
//        // if right then turn right, move forward 2 inches, put pixel next to team object, move back 2 inches, turn 180 degrees
//        if (team_object_position == 2) {
//            // if team object is in center
//            tad.pushTrayPixel(1500, tad.forward_distance, tad.reverse_distance);
//            // move right to avoid dislodging the pixel
//            tad.moveParallelToRightD(tad.DRIVE_SPEED, 18, 2.0);
//            //tad.moveParallelToRight(1200);
//            //turn left
//            tad.driveRobot(tad.TURN_SPEED, -tad.turn_distance, tad.turn_distance, 4.0);
//            // move Right toward the crossing bar
//            tad.moveParallelToRightD(tad.DRIVE_SPEED, 28, 4.0);
//            //tad.moveParallelToRight(2700);
//            // for the front side - move closer to april tag by crossing
//            tad.driveRobot(tad.DRIVE_SPEED, tad.front_cross_distance+18, tad.front_cross_distance+18, 5.0);
//            // move left back towards the April Tag
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 28, 3.0);
//            //tad.moveParallelToLeft(2100);
//        } else if (team_object_position == 1){
//            // if team object position is left
//            // move away from object
//            tad.moveParallelToRightD(tad.DRIVE_SPEED, tad.move_away_distance, 2.0);
//            //tad.moveParallelToRight(400);
//            //turn left
//            tad.driveRobot(tad.TURN_SPEED, -tad.turn_distance, tad.turn_distance, 3.0);
//            //move forward and drop the pixel
//            tad.pushTrayPixel(1500, (float) (tad.forward_distance+tad.move_away_adjustment), tad.reverse_distance );
//            // move right towards cross bar
//            tad.moveParallelToRightD(tad.DRIVE_SPEED, 27, 3.0);
//            //tad.moveParallelToRight(2400);
//            // for the front side - move closer to april tag by crossing
//            tad.driveRobot(tad.DRIVE_SPEED, tad.front_cross_distance+3, tad.front_cross_distance+3, 5.0);
//            //move left back towards April Tag
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 35, 4.0);
//            //tad.moveParallelToLeft(2400);
//        } else if (team_object_position == 3){
//            // if team object position is right;
//            // move little left before turning right
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, tad.move_away_distance, 2.0);
//            //tad.moveParallelToLeft(400);
//            //turn right
//            tad.driveRobot(tad.TURN_SPEED, tad.turn_distance, -tad.turn_distance, 3.0);
//            // move more left to ensure pixel is dropped on line
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 9, 2.0);
//            tad.pushTrayPixel(1500, (float) (tad.forward_distance+tad.move_away_adjustment-0.5),tad.reverse_distance-(float)2);
//            // get towards the cross bar
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 18, 3.0);
//            //tad.moveParallelToLeft(2600);
//            // turn 180 degrees towards the team object
//            tad.driveRobot(tad.TURN_SPEED,   (tad.turn_distance*2)-0.5, -(tad.turn_distance*2)-0.5, 6.0);
//            // for the front side - move closer to april tag by crossing
//            tad.driveRobot(tad.DRIVE_SPEED, tad.front_cross_distance, tad.front_cross_distance, 5.0);
//            tad.moveParallelToLeftD(tad.DRIVE_SPEED, 20, 3.0);
//            //tad.moveParallelToLeft(1800);
//        } else {
//            telemetry.addData("Not able  to find object"," object position %d ", team_object_position);
//            telemetry.update();
//            team_object_position = 2;
//            sleep(1000);
//        }
//        return team_object_position;
//    }
//}
