package org.firstinspires.ftc.teamcode.ebotssensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;

public class EbotsWebcam {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private String deviceName;
    private WebcamName webcamName = null;

    // if using for navigation, must locate camera from center of robot
    private float xTranslation;
    private float yTranslation;
    private float zTranslation;

    // these translations get set based on which side the camera faces
    // for these transformation, camera long side is aligned with robot x axis (fore-aft)
    // and short side of camera is facing y+, camera lens is facing up z+
    private float xAxisRotationDeg;
    private float yAxisRotationDeg = 0;
    private float zAxisRotationDeg;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public EbotsWebcam(HardwareMap hardwareMap, String deviceName, RobotSide facingRobotSide, float x, float y, float z){
        this.deviceName = deviceName;
        webcamName = hardwareMap.get(WebcamName.class, deviceName);

        this.xTranslation = x;
        this.yTranslation = y;
        this.zTranslation = z;

        this.xAxisRotationDeg = 90.0f;
        this.yAxisRotationDeg = 0.0f;

        if(facingRobotSide==RobotSide.FRONT){
            zAxisRotationDeg = 90.0f;
            this.yAxisRotationDeg = -10.0f;
        } else if (facingRobotSide == RobotSide.LEFT){
            zAxisRotationDeg = 180.0f;
        } else if (facingRobotSide == RobotSide.BACK){
            zAxisRotationDeg = -90.0f;
        } else {    //RobotSide.RIGHT)
            zAxisRotationDeg = 0.0f;
        }
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public String getDeviceName() {
        return deviceName;
    }

    public WebcamName getWebcamName() {
        return webcamName;
    }

    public float getxTranslation() {
        return xTranslation;
    }

    public float getyTranslation() {
        return yTranslation;
    }

    public float getzTranslation() {
        return zTranslation;
    }

    public float getxAxisRotationDeg() {
        return xAxisRotationDeg;
    }

    public float getyAxisRotationDeg() {
        return yAxisRotationDeg;
    }

    public float getzAxisRotationDeg() {
        return zAxisRotationDeg;
    }
}
