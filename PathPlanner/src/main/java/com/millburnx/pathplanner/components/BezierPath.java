package com.millburnx.pathplanner.components;

import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.geom.Path2D;
import java.util.ArrayList;

import com.millburnx.purePursuit.Utils.Point;

public class BezierPath {
    private final ArrayList<BezierPoint> points = new ArrayList<>();

    public void addPoint(BezierPoint point) {
        points.add(point);
    }

    public ArrayList<BezierPoint> getPoints() {
        return points;
    }

    public void draw(Graphics2D g2d, double ppi) {
        if (points.size() >= 2) {
            Path2D path = new Path2D.Double();
            BezierPoint startPoint = points.get(0);
            Point start = startPoint.getAnchor().times(ppi);
            path.moveTo(start.getX(), start.getY());

            for (int i = 1; i < points.size(); i++) {
                BezierPoint current = points.get(i);
                BezierPoint previous = points.get(i - 1);

                Point p2 = previous.getNextHandle().times(ppi);
                Point p3 = current.getPreviousHandle().times(ppi);
                Point p4 = current.getAnchor().times(ppi);

                path.curveTo(
                        p2.getX(), p2.getY(),
                        p3.getX(), p3.getY(),
                        p4.getX(), p4.getY()
                );
            }
            g2d.setColor(Color.WHITE);
            g2d.draw(path);
        }
    }

    public void drawHandles(Graphics2D g2d, double ppi) {
        for (int i = 0; i < points.size(); i++) {
            BezierPoint point = points.get(i);
            g2d.setColor(Color.GREEN);
            Point anchor = point.getAnchor().times(ppi);
            g2d.fillOval((int) (anchor.getX() - 5), (int) (anchor.getY() - 5), 10, 10);

            if (point.getPreviousHandle() != null) {
                g2d.setColor(Color.MAGENTA);
                Point handle = point.getPreviousHandle().times(ppi);
                g2d.drawLine((int) anchor.getX(), (int) anchor.getY(), (int) handle.getX(), (int) handle.getY());
                g2d.fillOval((int) (handle.getX() - 5), (int) (handle.getY() - 5), 10, 10);
            }
            if (point.getNextHandle() != null && i < points.size() - 1) {
                g2d.setColor(Color.CYAN);
                Point handle = point.getNextHandle().times(ppi);
                g2d.drawLine((int) anchor.getX(), (int) anchor.getY(), (int) handle.getX(), (int) handle.getY());
                g2d.fillOval((int) (handle.getX() - 5), (int) (handle.getY() - 5), 10, 10);
            }
        }
    }
}
