package org.firstinspires.ftc.teamcode.GoalDetector;

import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;

public class OpenCVHierchialContour {
    MatOfPoint contour;
    int idxNext, idxprev, idxFirstChild, idxparent;

    public OpenCVHierchialContour(MatOfInt hierarchy, ArrayList<MatOfPoint> contours, int i)
    {
        contour = contours.get(i);
        idxparent = parent(hierarchy.get(0,i));
        idxFirstChild = firstChild(hierarchy.get(0,i));
        idxprev = idxPrev(hierarchy.get(0,i));
        idxNext = idxNext(hierarchy.get(0,i));
    }

    int[] intGet(MatOfInt mat, int idx)
    {
        int[] arr = new int[4];

        mat.get(0, idx, arr);

        return arr;
    }

    static int idxNext(double[] val)
    {
        return (int) Math.round(val[0]);
    }

    static int idxPrev(double[] val)
    {
        return (int) Math.round(val[1]);
    }

    static int firstChild(double[] val)
    {
        return (int) Math.round(val[2]);
    }

    static int parent(double[] val)
    {
        return (int) Math.round(val[3]);
    }

    public boolean hasParent()
    {
        return idxparent >= 0;
    }
}
