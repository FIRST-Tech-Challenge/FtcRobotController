package com.acmerobotics.dashboard.canvas;

import com.google.gson.annotations.SerializedName;

public abstract class CanvasOp {
    public enum Type {
        @SerializedName("grid")
        GRID,

        @SerializedName("translate")
        TRANSLATE,

        @SerializedName("rotation")
        ROTATION,

        @SerializedName("scale")
        SCALE,

        @SerializedName("alpha")
        ALPHA,

        @SerializedName("circle")
        CIRCLE,

        @SerializedName("polygon")
        POLYGON,

        @SerializedName("polyline")
        POLYLINE,

        @SerializedName("spline")
        SPLINE,

        @SerializedName("stroke")
        STROKE,

        @SerializedName("fill")
        FILL,

        @SerializedName("strokeWidth")
        STROKE_WIDTH,


        @SerializedName("text")
        TEXT,

        @SerializedName("image")
        IMAGE;
    }

    private Type type;

    public CanvasOp(Type type) {
        this.type = type;
    }
}
