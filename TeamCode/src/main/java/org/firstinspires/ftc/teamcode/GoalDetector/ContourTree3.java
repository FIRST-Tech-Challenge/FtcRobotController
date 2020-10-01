package org.firstinspires.ftc.teamcode.GoalDetector;

import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class ContourTree3
{
    ArrayList<Contour> rootContours = new ArrayList<>();

    public ContourTree3(MatOfInt hierarchy, ArrayList<MatOfPoint> contours)
    {
        ArrayList<OpenCVHierchialContour> openCVHierchialContours = new ArrayList<>();

        for(int i = 0; i < contours.size(); i++)
        {
            openCVHierchialContours.add(new OpenCVHierchialContour(hierarchy, contours, i));
        }

        for(int i = 0; i < openCVHierchialContours.size(); i++)
        {
            if(!openCVHierchialContours.get(i).hasParent())
            {
                Contour parentContour = new Contour();

                rootContours.add(parentContour);

                parentContour.self = contours.get(i);
                parentContour.parent = null;
                parentContour.area = Imgproc.contourArea(parentContour.self);
                parentContour.originalListIdx = i;

                traverseTree(openCVHierchialContours, parentContour, parentContour.children);
            }
        }
    }

    public static void traverseTree(ArrayList<OpenCVHierchialContour> contours, Contour start, ArrayList<Contour> addChildrenTo)
    {
        int idxNextTarget = contours.get(start.originalListIdx).idxFirstChild;

        while (idxNextTarget >= 0)
        {
            Contour child = new Contour();
            child.self = contours.get(idxNextTarget).contour;
            child.area = Imgproc.contourArea(child.self);
            child.originalListIdx = idxNextTarget;
            child.parent = start;
            addChildrenTo.add(child);
            traverseTree(contours, child, child.children);

            idxNextTarget = contours.get(idxNextTarget).idxNext;
        }
    }

    public static void filterByArea(double minArea, ArrayList<Contour> list)
    {
        ArrayList<Contour> remove = new ArrayList<>();

        for(Contour c : list)
        {
            if(c.area < minArea)
            {
                remove.add(c);
            }
            else
            {
                filterByArea(minArea, c.children);
            }
        }

        list.removeAll(remove);
    }

    public static void filterBySqrtArea(double minSqrtArea, ArrayList<Contour> list)
    {
        ArrayList<Contour> remove = new ArrayList<>();

        for(Contour c : list)
        {
            if(Math.sqrt(c.area) < minSqrtArea)
            {
                remove.add(c);
            }
            else
            {
                filterByArea(minSqrtArea, c.children);
            }
        }

        list.removeAll(remove);
    }
}
