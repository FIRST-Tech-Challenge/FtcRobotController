package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;
@Config
public class Var_Blue {
    public enum DetectionTypes {
        DAY,
        NIGHT
    }

    public static double Day_Hhigh = 180, Day_Shigh = 255, Day_Vhigh = 255, Day_Hlow = 0, Day_Slow = 150, Day_Vlow = 150;
    public static double Night_Hhigh = 180, Night_Shigh = 255, Night_Vhigh = 255, Night_Hlow = 90, Night_Slow = 150, Night_Vlow = 100;
    public static int CV_kernel_pult_size = 5, Webcam_w = 640, Webcam_h = 480, CV_rect_x1 = 0, CV_rect_y1 = 0, CV_rect_x2 = 640, CV_rect_y2 = 480;
    public static DetectionTypes CV_detectionType = DetectionTypes.DAY;
    public static Scalar scalar = new Scalar(0,0,0);
}
