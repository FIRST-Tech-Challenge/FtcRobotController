package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class SkystoneCV {
    SkystoneWindowDetector detector;
    OpenCvCamera camera;
    String CAMERA_NAME;
    OpMode opMode;

    double width = 30;
    Point leftRectCorner = new Point(20, 130);
    Point centerRectCorner = new Point(90, 130);
    Point rightRectCorner = new Point(170, 130);

    public enum CameraType { WEBCAM, FRONT, BACK }

    public enum StonePosition { LEFT, CENTER, RIGHT, UNKNOWN }

    public SkystoneCV (String cameraName, Point leftRectCorner, Point centerRectCorner, Point rightRectCorner, LinearOpMode opMode) {
        detector = new SkystoneWindowDetector(opMode);
        CAMERA_NAME = cameraName;
        this.opMode = opMode;
        detector.setWindows(new Rect((int) leftRectCorner.x, (int) leftRectCorner.y, (int)width, (int)width),new Rect((int) centerRectCorner.x, (int) centerRectCorner.y, (int)width, (int )width), new Rect((int) rightRectCorner.x, (int) rightRectCorner.y, (int)width, (int)width));

        this.leftRectCorner = leftRectCorner;
        this.centerRectCorner = centerRectCorner;
        this.rightRectCorner = rightRectCorner;
    }


    public void init(CameraType cameraType) {
        detector.useDefaults();
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        if (cameraType == CameraType.WEBCAM) {
            camera = new OpenCvWebcam(opMode.hardwareMap.get(WebcamName.class, CAMERA_NAME), cameraMonitorViewId);
        } else if (cameraType == CameraType.FRONT){
            camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT);
        } else {
            camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        }
        camera.openCameraDevice();
        camera.setPipeline(detector);
        //todo: check width and height
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public StonePosition getSkystonePosition() {
        return detector.skystonePosition;
    }

    //very bad joystick window moving method
    public void setWindows(Gamepad gamepad) {
        //hold down "x" to move left box
        //hold down "a" to move center box
        //hold down "b" to move right box

        //inc is number of pixels that box will move per cycle (1 is very slow and 10 is fast)

        double inc = 5;
        if (gamepad.left_stick_x > 0) {
            width += inc;
        } else if (gamepad.left_stick_x < 0) {
            width -= inc;
        }

        if (gamepad.right_stick_x < 0) {
            if (gamepad.x) {
                leftRectCorner.x += inc;
            } if (gamepad.a) {
                centerRectCorner.x += inc;
            } if (gamepad.b) {
                rightRectCorner.x += inc;
            }
        } else if (gamepad.right_stick_x > 0) {
            if (gamepad.x) {
                leftRectCorner.x -= inc;
            } if (gamepad.a) {
                centerRectCorner.x -= inc;
            } if (gamepad.b) {
                rightRectCorner.x -= inc;
            }
        }

        if (-gamepad.right_stick_y < 0) {
            if (gamepad.x) {
                leftRectCorner.y += inc;
            } if (gamepad.a) {
                centerRectCorner.y += inc;
            } if (gamepad.b) {
                rightRectCorner.y += inc;
            }
        } else if (-gamepad.right_stick_y > 0) {
            if (gamepad.x) {
                leftRectCorner.y -= inc;
            } if (gamepad.a) {
                centerRectCorner.y -= inc;
            } if (gamepad.b) {
                rightRectCorner.y -= inc;
            }
        }
        detector.setWindows(new Rect((int) leftRectCorner.x, (int) leftRectCorner.y, (int)width, (int)width),new Rect((int) centerRectCorner.x, (int) centerRectCorner.y, (int)width, (int)width), new Rect((int) rightRectCorner.x, (int) rightRectCorner.y, (int)width, (int)width));
    }
}


class SkystoneWindowDetector extends DogeCVDetector {
    SkystoneCV.StonePosition skystonePosition;
    OpMode opMode;

    //random placeholder numbers
    Rect leftRect = new Rect(100, 100, 100, 100);
    Rect centerRect = new Rect(100, 100, 100, 100);
    Rect rightRect = new Rect(100, 100, 100, 100);

    List<Stone> stones = new ArrayList<>();

    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();

    public SkystoneWindowDetector (OpMode opMode) {
        this.opMode = opMode;
        stones.add(new Stone(SkystoneCV.StonePosition.LEFT, leftRect));
        stones.add(new Stone(SkystoneCV.StonePosition.CENTER, centerRect));
        stones.add(new Stone(SkystoneCV.StonePosition.RIGHT, rightRect));
    }

    @Override
    public Mat process(Mat input) {
        input.copyTo(workingMat);
        input.copyTo(displayMat);

        stones.get(0).update(displayMat); //just in case stone at pos 0 from last cycle has lower yellow than all this cycle
        Stone skystone = stones.get(0);

        for (Stone stone : stones) {
            stone.update(workingMat);
            Imgproc.rectangle(displayMat, stone.rect, new Scalar(255,255,255), 1, Imgproc.LINE_4, 0);
            Imgproc.putText(displayMat, "" + stone.position, new Point(stone.rect.x, stone.rect.y), Imgproc.FONT_HERSHEY_PLAIN, .1, new Scalar(255, 255, 255));
            opMode.telemetry.addData("" + stone.position, stone.getYellow());
            if (stone.getYellow() < skystone.getYellow()) skystone = stone;
        }

        skystonePosition = skystone.position;

        return displayMat;
    }

    public void setWindows (Rect leftRect, Rect centerRect, Rect rightRect) {
        getStone(SkystoneCV.StonePosition.LEFT).setRect(leftRect);
        getStone(SkystoneCV.StonePosition.CENTER).setRect(centerRect);
        getStone(SkystoneCV.StonePosition.RIGHT).setRect(rightRect);
    }

    public void printWindows (OpMode opMode) {
        opMode.telemetry.addData("LEFT Window", getStone(SkystoneCV.StonePosition.LEFT).rect);
        opMode.telemetry.addData("CENTER Window", getStone(SkystoneCV.StonePosition.CENTER).rect);
        opMode.telemetry.addData("RIGHT Window", getStone(SkystoneCV.StonePosition.RIGHT).rect);
    }

    public Stone getStone (SkystoneCV.StonePosition position) {
        for (Stone stone : stones) {
            if (stone.position == position) return stone;
        }
        return stones.get(0); //defaults to first stone (not the best)
    }

    @Override
    public void useDefaults() { }
}

class Stone {
    public Mat mat;
    public Rect rect;
    Scalar mean;
    public SkystoneCV.StonePosition position;

    public Stone (SkystoneCV.StonePosition position, Rect rect) {
        this.position = position;
        this.rect = rect;
    }

    //averages red and green values - could probably be improved
    public double getYellow () {
        return (mean.val[0] + mean.val[1])/2;
    }

    public void setRect (Rect rect) {
        this.rect = rect;
    }

    public void update (Mat workingMat) {
        mat = new Mat(workingMat, rect);
        mean = Core.mean(mat);
    }
}
