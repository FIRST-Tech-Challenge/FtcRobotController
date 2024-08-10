//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package android.util;

public final class Size {
    private int width, height;

    public Size(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public boolean equals(Object obj) {
        return obj == this;
    }

    public String toString() {
        return String.format("Size(%d, %d)", width, height);
    }

    public static Size parseSize(String string) throws NumberFormatException {
        throw new RuntimeException("Unimplemented!");
    }

    public int hashCode() {
        throw new RuntimeException("Unimplemented!");
    }
}
