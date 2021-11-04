package org.firstinspires.ftc.teamcode.Util.CoordinateSystem.PathFinder;

import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Coordinate;

// Node class for convenience
public class Node extends Coordinate implements Comparable {
    public Node parent;

    public Node(Node parent, int xPos, int yPos, double g, double h) {
        super(xPos, yPos, g, h);
        this.parent = parent;
    }

    // Compare by f value (g + h)
    @Override
    public int compareTo(java.lang.Object o) {
        Node that = (Node) o;
        return (int) ((this.g + this.h) - (that.g + that.h));
    }
}
