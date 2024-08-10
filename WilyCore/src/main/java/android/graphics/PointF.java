//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package android.graphics;

import androidx.annotation.RecentlyNonNull;

public class PointF  {
    public float x, y = 0;

    public PointF() { }
    public PointF(float x, float y) {
        this.x = x; this.y = y;
    }

    public final void set(float x, float y) {
        this.x = x; this.y = y;
    }

    public final void set(@RecentlyNonNull PointF p) {
        this.x = p.x;
        this.y = p.y;
    }

    public final void negate() {
        x = -x;
        y = -x;
    }

    public final void offset(float dx, float dy) {
        x += dx;
        y += dy;
    }

    public final boolean equals(float x, float y) {
        return (this.x == x) && (this.y == y);
    }

    public boolean equals(Object o) {
        if (!(o instanceof PointF))
            return false;
        PointF other = (PointF) o;
        return (other.x == x) && (other.y == y);
    }

    public int hashCode() {
        throw new RuntimeException("Stub!");
    }

    public String toString() {
        throw new RuntimeException("Stub!");
    }

    public final float length() {
        return (float) Math.hypot(x, y);
    }

    public static float length(float x, float y) {
        return (float) Math.hypot(x, y);
    }
}
