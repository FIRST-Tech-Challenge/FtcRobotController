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

    public void draw(Graphics2D g2d) {
        if (points.size() >= 2) {
            Path2D path = new Path2D.Double();
            BezierPoint startPoint = points.get(0);
            path.moveTo(startPoint.getAnchor().getX(), startPoint.getAnchor().getY());

            for (int i = 1; i < points.size(); i++) {
                BezierPoint current = points.get(i);
                BezierPoint previous = points.get(i - 1);

                path.curveTo(
                        previous.getNextHandle().getX(), previous.getNextHandle().getY(),
                        current.getPreviousHandle().getX(), current.getPreviousHandle().getY(),
                        current.getAnchor().getX(), current.getAnchor().getY()
                );
            }
            g2d.setColor(Color.WHITE);
            g2d.draw(path);
        }
    }

    public void drawHandles(Graphics2D g2d) {
        for (int i = 0; i < points.size(); i++) {
            BezierPoint point = points.get(i);
            g2d.setColor(Color.GREEN);
            g2d.fillOval((int) (point.getAnchor().getX() - 5), (int) (point.getAnchor().getY() - 5), 10, 10);

            if (point.getPreviousHandle() != null) {
                g2d.setColor(Color.MAGENTA);
                Point anchor = point.getAnchor();
                Point handle = point.getPreviousHandle();
                g2d.drawLine((int) anchor.getX(), (int) anchor.getY(), (int) handle.getX(), (int) handle.getY());
                g2d.fillOval((int) (handle.getX() - 5), (int) (handle.getY() - 5), 10, 10);
            }
            if (point.getNextHandle() != null && i < points.size() - 1) {
                g2d.setColor(Color.CYAN);
                Point anchor = point.getAnchor();
                Point handle = point.getNextHandle();
                g2d.drawLine((int) anchor.getX(), (int) anchor.getY(), (int) handle.getX(), (int) handle.getY());
                g2d.fillOval((int) (handle.getX() - 5), (int) (handle.getY() - 5), 10, 10);
            }
        }
    }
}
