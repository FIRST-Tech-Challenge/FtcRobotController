package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class VisionPoseConverter {
    private double xOffset = 0;
    private double yOffset = 0;
    private Vector2d[] values = {new Vector2d(0, 0), new Vector2d(0, 0),new Vector2d(0, 0),new Vector2d(0, 0),
            new Vector2d(0, 0),new Vector2d(0, 0),new Vector2d(0, 0)
            ,new Vector2d(0, 0),new Vector2d(1.5*23.5, 3*23.5-1),new Vector2d(1.5*23.5-5.5, 3*23.5-1)
            ,new Vector2d(-1.5*23.5, 3*23.5-1),new Vector2d(-1.5*23.5+5.5, 3*23.5-1)};
    private double[][] directions = {{-1, 1}, {-1, 1}};

    public VisionPoseConverter(double p_xOffset, double p_yOffset) {
        xOffset = p_xOffset;
        yOffset = p_yOffset;
    }

    public Vector2d calcPoseVector(double p_x, double p_y, int p_ind) {
        Vector2d in = new Vector2d(p_x, p_y);
        return values[p_ind].plus(new Vector2d(p_x * directions[p_ind][0], p_y * directions[p_ind][1]));
    }

}
