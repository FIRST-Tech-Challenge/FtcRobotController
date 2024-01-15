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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;

@Autonomous(name="Autonomous", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class autonomous extends LinearOpMode {

    private static final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    //private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    //private static final String LABEL_FIRST_ELEMENT = "Skystone";

    //private VuforiaLocalizer vuforia;
    //private TFObjectDetector tfod;

    //private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    
    static int pathSide = 0; // 0 = red, 1 = blue;
    int pathSelected = 0;
    int selectState = 0;
    int selectObject = 0;
    private boolean[] button = {false, false, false, false};
    

    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware H = new RobotHardware();
    private MecanumWheelDriverV2 drive = new MecanumWheelDriverV2(H);;
    private ExecutorService pool = Executors.newFixedThreadPool(2);
    private AutonomousPaths paths = new AutonomousPaths(drive, H, this);
    
    boolean dropBottom = false;
    boolean dropTop = false;
    long duckServoStartTime = 0;
    //double[] point;
    static int color;

    @Override
    public void runOpMode() {
    
        RobotHardware.headingSave = 0;
        H.setCameraEnable(true);
        H.setXYEncoderEnable(true);
        H.init(hardwareMap, this);
        pool.execute(H);
        pool.execute(drive);
        //initVuforia();
        //ColorReader detector = new ColorReader(vuforia);
        
        H.setGrabber(false);
        H.LEDMode = 3;
        //QRdetector detector = new QRdetector(H);
    
        /*H.rampServo.setPosition(1);
        H.rampServo.setPosition(0.5);
        H.collectorServo.setPosition(0);
        H.wheelLift[0].setPosition(1);
        H.wheelLift[1].setPosition(0);
        H.wheelLift[2].setPosition(0);
        H.wheelLift[3].setPosition(1);*/
        /*while (!isStopRequested() && !isStarted()) {
            telemetry.addData("cone", detector.count() + 1);
            telemetry.update();
        }*/
        
        while (!isStopRequested() && !isStarted()) {
            
            if (gamepad1.dpad_up) {
                if (!button[0] && selectObject > 0) {
                    selectObject --;
                    button[0] = true;
                }
            }
            else button[0] = false;
    
            if (gamepad1.dpad_down) {
                if (!button[1] && selectObject < paths.menus[selectState].length - 2) {
                    selectObject ++;
                    button[1] = true;
                }
            }
            else button[1] = false;
            
            if (gamepad1.b) {
                if (!button[2] && selectState > 0) {
                    selectState--;
                    selectObject = 0;
                    button[2] = true;
                }
            }
            else button[2] = false;
    
            if (gamepad1.a) {
                if (!button[3] && selectState < 2) {
                    if (selectState == 0) {
                        pathSide = selectObject;
                    }
                    if (selectState == 1) {
                        pathSelected = selectObject;
                    }
                    selectState++;
                    selectObject = 0;
    
                    button[3] = true;
                }
            }
            else button[3] = false;
            
            if (selectState != 2) telemetry.addLine(paths.menus[selectState][0]);
            else telemetry.addData(paths.menus[selectState][0], paths.menus[0][pathSide + 1] + " " + paths.menus[1][pathSelected + 1]);
            
            for (int i = 0; i < paths.menus[selectState].length - 1; i ++) {
                if (i == selectObject) {
                    telemetry.addLine( "- " + paths.menus[selectState][i+1] + " -");
                } else {
                    telemetry.addLine(paths.menus[selectState][i+1]);
                }
            }
            
            //point = detector.getCenterPoint();
            
            //telemetry.addData("","");
            //telemetry.addData("\npoint x", point[0]);
            //color = detector.count();
            telemetry.update();
        }
    
        //aimBot = new AimBot(H, drive, pool, this);
        //RingCounter         ring     = new RingCounter(aimBot.vuforia);
    
        //H.grabberServo.setPosition(H.GRABBER_SERVO_MIN);
    
        waitForStart();
        
        //color = detector.count();
        //vuforia.close();
        H.LED.turnAllOff();
        H.LEDMode = 0;
        
        /*H.wheelLift[0].setPosition(0);
        H.wheelLift[1].setPosition(1);
        H.wheelLift[2].setPosition(1);
        H.wheelLift[3].setPosition(0);*/
        
        //point = detector.getCenterPoint();
        
        telemetry.addData(paths.menus[2][0], paths.menus[0][pathSide + 1] + " " + paths.menus[1][pathSelected + 1]);
        //telemetry.addData("point x", point[0]);
        telemetry.update();
        
        //detector.shutDown();
        
        paths.runPath(pathSide, pathSelected);
        
        while (opModeIsActive()) {
            idle();
        }
        H.LED.turnAllOff();
        drive.stop();
        H.saveHeading();
        pool.shutdown();
    }
    
    void deployRamp() {
        
        long servoStartTime;
        
        //H.rampServo.setPosition(1);
        servoStartTime = runtime.time(TimeUnit.MILLISECONDS);
    
        /*if (point[0] - 320 < - 100) {
            dropBottom = true;
            while (runtime.time(TimeUnit.MILLISECONDS) < servoStartTime + 1300 && !isStopRequested()) {
                idle();
            }
        } else if (point[0] - 320 > 100) {
            dropTop = true;
            while (runtime.time(TimeUnit.MILLISECONDS) < servoStartTime + 3350 && !isStopRequested()) {
                idle();
            }
        } else {
            while (runtime.time(TimeUnit.MILLISECONDS) < servoStartTime + 1300 && !isStopRequested()) {
                idle();
            }
        }*/
        //H.rampServo.setPosition(0.5);
        if (dropBottom) {
            //H.rampServo.setPosition(0);
            servoStartTime = runtime.time(TimeUnit.MILLISECONDS);
            while (runtime.time(TimeUnit.MILLISECONDS) < servoStartTime + 1250 && !isStopRequested()) {
                idle();
            }
            //H.rampServo.setPosition(0.5);
        }
        
    }
    
    void spinDuck() {
    
        duckServoStartTime = runtime.time(TimeUnit.MILLISECONDS);
        //H.duckServo.setPosition(-pathSide + 1);
    
        while (runtime.time(TimeUnit.MILLISECONDS) < duckServoStartTime + 3000 && !isStopRequested()) {
            idle();
        }
        //H.duckServo.setPosition(0.5);
        
    }
    
    void dropFreight() {
    
        if (dropBottom) {
            drive.StrafeDistanceAngleMove(180,2,0.6,1);
            drive.startActions();
            drive.waitForMoveDone();
        }
        if (dropTop) {
            drive.StrafeDistanceAngleMove(0,2,0.6,1);
            drive.startActions();
            drive.waitForMoveDone();
        }
        /*
        H.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        H.liftMotor.setTargetPosition(270);
        H.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.liftMotor.setPower(1);
        while (H.liftMotor.isBusy()) {
            idle();
        }
        H.liftMotor.setTargetPosition(0);*/
        sleep(500);
        //H.rampServo.setPosition(1);
        
    }
    /*
    void initVuforia() {
    
        int cameraMonitorViewId = H.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", H.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    
        parameters.vuforiaLicenseKey = H.VUFORIA_KEY;
        //parameters.cameraName = H.webcam;
        //parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
    
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }*/

}