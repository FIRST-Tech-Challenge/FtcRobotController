package org.firstinspires.ftc.teamcode;

import android.util.Log;

import java.util.concurrent.TimeUnit;

import static android.content.ContentValues.TAG;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonomousPaths {
    
    MecanumWheelDriverV2 drive;
    RobotHardware H;
    autonomous auto;
    
    final String[][] menus = {{"Field Side:", "Red", "Blue"},{"Autonomous Path:","Park", "One Cone Right", "One Cone Left"}, {"Path Selected"}};
    
    AutonomousPaths(MecanumWheelDriverV2 drive, RobotHardware H, autonomous auto) {
        
        this.drive = drive;
        this.H = H;
        this.auto = auto;
        
    }
    
    void runPath(int side, int path) {
    
        Log.d(TAG,"path: " + path + " side: " + side);
        if (side == 0) { // red
            
            switch (path) {
                case 0:
                    Park();
                    break;
                case 1:
                    OneConeRight();
                    break;
                case 2:
                    OneConeLeft();
                    break;
                case 3:
                    TwoConeRight();
                    break;
                case 4:
                    TwoConeLeft();
                    break;
                case 5:
                    Square();
                    break;
                default:
                    break;
            }
            
        } else { // blue
    
            switch (path) {
                case 0:
                    Park();
                    break;
                case 1:
                    OneConeRight();
                    break;
                case 2:
                    OneConeLeft();
                    break;
                case 3:
                    TwoConeRight();
                    break;
                case 4:
                    TwoConeLeft();
                    break;
                case 5:
                    Square();
                    break;
                default:
                    break;
            }
            
        }
        
    }
    
    void OneConeRight() {
        
        H.tracker.setInitialPosition(108,6);
        
        H.setLiftPos((int)H.MAX_LIFT_POS);
    
        drive.StrafeDistanceMove(0,63,0.8,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(180,5,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        drive.StrafeDistanceMove(90,12,0.7,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(0, 0.2, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafePointMove(0.35, 96, 65.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setGrabber(true);
    
        drive.StrafeDistanceMove(180, 6,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setLiftPos(0);
    
        switch (auto.color) {
            case 0:
                drive.StrafeDistanceMove(90,12,0.5,1);
                break;
            case 1:
                drive.StrafeDistanceMove(-90,12,0.5,1);
                break;
            case 2:
                drive.StrafeDistanceMove(-90,36,0.5,1);
                break;
            default:
                break;
        }
        drive.startActions();
        drive.waitForMoveDone();
    }
    
    void OneConeLeft() {
        
        H.tracker.setInitialPosition(36,6);
        
        H.setLiftPos((int)H.MAX_LIFT_POS);
        
        drive.StrafeDistanceMove(0,63,0.8,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(180,5,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(-90,12,0.7,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(0, 0.2, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafePointMove(0.35, 48, 65.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setGrabber(true);
        
        drive.StrafeDistanceMove(180, 6,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setLiftPos(0);
        
        switch (auto.color) {
            case 0:
                drive.StrafeDistanceMove(90,36,0.5,1);
                break;
            case 1:
                drive.StrafeDistanceMove(90,12,0.5,1);
                break;
            case 2:
                drive.StrafeDistanceMove(-90,12,0.5,1);
                break;
            default:
                break;
        }
        drive.startActions();
        drive.waitForMoveDone();
    }
    
    void TwoConeRight() {
        
        H.tracker.setInitialPosition(108,6);
        
        H.setLiftPos((int)H.MAX_LIFT_POS);
        
        drive.StrafeDistanceMove(0,63,0.8,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(180,5,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(90,12,0.7,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(0, 0.2, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafePointMove(0.35, 96, 65.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setGrabber(true);
        
        drive.StrafeDistanceMove(180, 6,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setLiftPos(0);
        
        switch (auto.color) {
            case 0:
                drive.StrafeDistanceMove(90,12,0.5,1);
                break;
            case 1:
                drive.StrafeDistanceMove(-90,12,0.5,1);
                break;
            case 2:
                drive.StrafeDistanceMove(-90,36,0.5,1);
                break;
            default:
                break;
        }
        drive.startActions();
        drive.waitForMoveDone();
    }
    
    void TwoConeLeft() {
        
        H.tracker.setInitialPosition(36,6);
        
        H.setLiftPos((int)H.MAX_LIFT_POS);
        
        drive.StrafeDistanceMove(0,63,0.8,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(180,5,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(-90,12,0.7,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(0, 0.2, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafePointMove(0.35, 48, 65.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setGrabber(true);
        
        drive.StrafeDistanceMove(180, 6,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        H.setLiftPos(0);
        
        switch (auto.color) {
            case 0:
                drive.StrafeDistanceMove(90,36,0.5,1);
                break;
            case 1:
                drive.StrafeDistanceMove(90,12,0.5,1);
                break;
            case 2:
                drive.StrafeDistanceMove(-90,12,0.5,1);
                break;
            default:
                break;
        }
        drive.startActions();
        drive.waitForMoveDone();
    }
    
    void Park() {
        drive.StrafeDistanceMove(0,30,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        switch (auto.color) {
            case 0:
                drive.StrafeDistanceMove(90,24,0.5,1);
                drive.startActions();
                drive.waitForMoveDone();
                break;
            case 1:
                break;
            case 2:
                drive.StrafeDistanceMove(-90,24,0.5,1);
                drive.startActions();
                drive.waitForMoveDone();
                break;
            default:
                break;
        }
    }
    
    void Square() {
    
        drive.StrafeDistanceMove(0,40,0.5,1);
        drive.HeadingRotate(-90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        drive.StrafeDistanceMove(-90,40,0.5,1);
        drive.HeadingRotate(180, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        drive.StrafeDistanceMove(180,40,0.5,1);
        drive.HeadingRotate(90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(90,40,0.5,1);
        drive.HeadingRotate(0, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueDuckA() {
        
        drive.StrafeDistanceMove(53,30,0.5,1);
        drive.startActions();
        
        auto.deployRamp();
        
        drive.waitForMoveDone();
        auto.dropFreight();
        drive.StrafeDistanceMove(-90,45,0.5,1);
        drive.HeadingRotate(-90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        long startTime = H.runtime.time(TimeUnit.MILLISECONDS);
        while (H.rightDistance > 8 && !auto.isStopRequested() && startTime + 1000 < H.runtime.time(TimeUnit.MILLISECONDS)) {
            auto.telemetry.addLine("dis: " + H.rightDistance);
            auto.telemetry.update();
            drive.StrafePowerMove(-90, 0.4,1);
            drive.startActions();
        }
        drive.stop();
        drive.StrafePowerMove(-90, 0.15,1);
        
        auto.spinDuck();
        drive.stop();
        
        drive.StrafeDistanceMove(-10, 23, 0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void RedDuckA() {
        
        drive.StrafeDistanceMove(-53,30,0.5,1);
        drive.startActions();
    
        auto.deployRamp();
    
        drive.waitForMoveDone();
        auto.dropFreight();
        drive.StrafeDistanceMove(90,45,0.5,1);
        drive.HeadingRotate(180, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(180,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        long startTime = H.runtime.time(TimeUnit.MILLISECONDS);
        while (H.frontDistance > 8 && !auto.isStopRequested() && startTime + 1000 < H.runtime.time(TimeUnit.MILLISECONDS)) {
            auto.telemetry.addLine("dis: " + H.frontDistance);
            auto.telemetry.update();
            drive.StrafePowerMove(0, 0.4,1);
            drive.startActions();
        }
        drive.stop();
        //drive.StrafePowerMove(0, 0.15,1);
    
        auto.spinDuck();
        //drive.stop();
    
        drive.StrafeDistanceMove(10, 23, 0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueDropB() {
        
        drive.StrafeDistanceMove(5,39,0.5,1);
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        
        auto.deployRamp();
        
        drive.waitForMoveDone();
        
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        auto.dropFreight();
    
    
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(180, 19, 0.5,1);
        //H.collectorServo.setPosition(1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(90,41,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void RedDropB() {
        
        drive.StrafeDistanceMove(-5,39,0.5,1);
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
    
        auto.deployRamp();
    
        drive.waitForMoveDone();
    
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        auto.dropFreight();
    
    
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(180, 19, 0.5,1);
        //H.collectorServo.setPosition(1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(-90,41,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueParkB() {
    
        drive.StrafeDistanceMove(5,14,0.5,1);
        drive.startActions();
        //H.collectorServo.setPosition(1);
        drive.waitForMoveDone();
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        //drive.HeadingRotate(-90,0.5,1);
        drive.StrafeDistanceMove(90,38,0.4,1);
        drive.startActions();
        drive.waitForMoveDone();
    }
    
    void RedParkB() {
    
        drive.StrafeDistanceMove(-5,14,0.5,1);
        drive.startActions();
        //H.collectorServo.setPosition(1);
        drive.waitForMoveDone();
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        //drive.HeadingRotate(-90,0.5,1);
        drive.StrafeDistanceMove(-90,38,0.4,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueParkA() {
    
        drive.StrafeDistanceMove(0,40,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
    }
    
    void RedParkA() {
        
        drive.StrafeDistanceMove(0,40,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueDuckOnly() {
    
        drive.StrafeDistanceMove(-55,31,0.5,1);
        drive.HeadingRotate(-90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        while (H.rightDistance > 12 && !auto.isStopRequested()) {
            drive.StrafePowerMove(180, 0.4,1);
            drive.startActions();
        }
        drive.stop();
    
        auto.spinDuck();
        
        drive.StrafeDistanceMove(0, 24,0.6,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void RedDuckOnly() {
    
        drive.StrafeDistanceMove(55,31,0.5,1);
        drive.HeadingRotate(90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        while (H.rightDistance > 12 && !auto.isStopRequested()) {
            drive.StrafePowerMove(-180, 0.4,1);
            drive.startActions();
        }
        drive.stop();
    
        auto.spinDuck();
    
        drive.StrafeDistanceMove(0, 24,0.6,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
}
