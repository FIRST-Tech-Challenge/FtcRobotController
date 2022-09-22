package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.*;

public class Odometry {
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor perpendicularEncoder;
    private double trackWidth;
    private double forwardOffset;

    private double prev_left_encoder_pos = 0.0;
    private double prev_right_encoder_pos = 0.0;
    private double prev_perpendicular_encoder_pos = 0.0;
    private double x_pos = 0.0;
    private double y_pos = 0.0;
    private double heading = 0.0;

    // instantiate odometry class
    public Odometry(DcMotor getLeftEncoder,
                    DcMotor getRightEncoder,
                    DcMotor getPerpEncoder) {
        leftEncoder = getLeftEncoder;
        rightEncoder = getRightEncoder;
        perpendicularEncoder = getPerpEncoder;
        trackWidth = 20; // get actual measurement
        forwardOffset = 20; // get actual measurement
    }

    // run odometry and return list of doubles in form [x, y, heading]
    public void runOdom(){
        double left_encoder_pos = leftEncoder.getCurrentPosition();
        double right_encoder_pos = rightEncoder.getCurrentPosition();
        double perpendicular_encoder_pos = perpendicularEncoder.getCurrentPosition();

        double delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        double delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        double delta_perpendicular_encoder_pos = perpendicular_encoder_pos - prev_perpendicular_encoder_pos;

        double phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackWidth;
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        double delta_perp_pos = delta_perpendicular_encoder_pos - forwardOffset * phi;

        double delta_x = delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
        double delta_y = delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading);

        x_pos += delta_x;
        y_pos += delta_y;
        heading += phi;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_perpendicular_encoder_pos = perpendicular_encoder_pos;
    }

    public double getX() {
        return x_pos;
    }

    public double getY() {
        return y_pos;
    }

    public double getHeading() {
        return heading;
    }
    
    public void setX(double value) {
        x_pos = value;
    }
    
    public void setY(double value) {
        y_pos = value;
    }
    
    public void setHeading(double value) {
        heading = value;
    }
}
