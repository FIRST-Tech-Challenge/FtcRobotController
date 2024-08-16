package com.millburnx.pathplanner.components;

import java.awt.*;
import java.awt.geom.Path2D;
import java.util.ArrayList;

public class BezierPath {
    private final ArrayList<BezierPoint> points = new ArrayList<>();

    public void addPoint(BezierPoint point) {
        points.add(point);
    }

    public ArrayList<BezierPoint> getPoints() {
        return points;
    }

    public void draw(Graphics2D g2d) {
        if (points.size() >= 2) {
            Path2D path = new Path2D.Double();
            BezierPoint startPoint = points.get(0);
            path.moveTo(startPoint.getAnchor().x, startPoint.getAnchor().y);

            for (int i = 1; i < points.size(); i++) {
                BezierPoint current = points.get(i);
                BezierPoint previous = points.get(i - 1);

                path.curveTo(previous.getHandle1().x, previous.getHandle1().y, current.getHandle2().x, current.getHandle2().y, current.getAnchor().x, current.getAnchor().y);
            }
            g2d.setColor(Color.WHITE);
            g2d.draw(path);
        }
    }

    public void drawHandles(Graphics2D g2d) {
        for (int i = 0; i < points.size(); i++) {
            BezierPoint point = points.get(i);
            g2d.setColor(Color.GREEN);
            g2d.fillOval(point.getAnchor().x - 5, point.getAnchor().y - 5, 10, 10);

            if (point.getHandle1() != null) {
                g2d.setColor(Color.MAGENTA);
                g2d.drawLine(point.getAnchor().x, point.getAnchor().y, point.getHandle1().x, point.getHandle1().y);
                g2d.fillOval(point.getHandle1().x - 5, point.getHandle1().y - 5, 10, 10);
            }
            if (point.getHandle2() != null && i < points.size() - 1) {
                g2d.setColor(Color.CYAN);
                g2d.drawLine(point.getAnchor().x, point.getAnchor().y, point.getHandle2().x, point.getHandle2().y);
                g2d.fillOval(point.getHandle2().x - 5, point.getHandle2().y - 5, 10, 10);
            }
        }
    }
}
