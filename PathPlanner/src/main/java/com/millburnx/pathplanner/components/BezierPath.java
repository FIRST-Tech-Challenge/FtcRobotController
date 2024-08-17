package com.millburnx.pathplanner.components;

import java.awt.*;
import java.awt.geom.Path2D;
import java.util.ArrayList;

import com.millburnx.utils.Vec2d;

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
            Vec2d start = startPoint.getAnchor().times(ppi);
            path.moveTo(start.getX(), start.getY());

            for (int i = 1; i < points.size(); i++) {
                BezierPoint current = points.get(i);
                BezierPoint previous = points.get(i - 1);

                Vec2d p2 = previous.getNextHandle().times(ppi);
                Vec2d p3 = current.getPreviousHandle().times(ppi);
                Vec2d p4 = current.getAnchor().times(ppi);

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
            renderPoint(g2d, ppi, point.getAnchor(), 10/ppi);

            if (point.getPreviousHandle() != null) {
                g2d.setColor(Color.MAGENTA);
                renderLine(g2d, ppi, point.getAnchor(), point.getPreviousHandle());
                renderPoint(g2d, ppi, point.getPreviousHandle(), 10/ppi);
            }
            if (point.getNextHandle() != null && i < points.size() - 1) {
                g2d.setColor(Color.CYAN);
                renderLine(g2d, ppi, point.getAnchor(), point.getNextHandle());
                renderPoint(g2d, ppi, point.getNextHandle(), 10/ppi);
            }
        }
    }

    public void renderLine(Graphics2D g2d, double ppi, Vec2d p1, Vec2d p2) {
        Vec2d start = p1.times(ppi);
        Vec2d end = p2.times(ppi);
        g2d.drawLine((int) start.getX(), (int) start.getY(), (int) end.getX(), (int) end.getY());
    }

    public void renderPoint(Graphics2D g2d, double ppi, Vec2d point, double size) {
        Vec2d center = point.times(ppi);
        Vec2d sizeP = new Vec2d(size, size).times(ppi);
        Vec2d origin = center.minus(sizeP.div(2));
        g2d.fillOval((int) origin.getX(), (int) origin.getY(), (int) sizeP.getX(), (int) sizeP.getY());
    }
}
