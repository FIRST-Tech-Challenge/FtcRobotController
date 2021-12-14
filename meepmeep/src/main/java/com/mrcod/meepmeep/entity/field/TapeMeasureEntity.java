package com.mrcod.meepmeep.entity.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.entity.BotEntity;
import com.noahbres.meepmeep.core.entity.ThemedEntity;
import com.noahbres.meepmeep.core.util.FieldUtil;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;

public class TapeMeasureEntity extends BasicThemedEntity {
    private static final Color color = new Color(255, 218, 104);
    private final double maxLength;
    private final double speed;

    private double length = 0;
    private boolean extending = false;
    private Pose2d pose;

    public TapeMeasureEntity(MeepMeep meepMeep, Pose2d pose, double maxLength, double speed) {
        super(meepMeep, "TAPE_MEASURE_FIELD_ENTITY");
        this.maxLength = maxLength;
        this.speed = speed / 1000000000;
        this.pose = pose;
    }

    @Override
    public void render(@NotNull Graphics2D graphics2D, int canvasWidth, int canvasHeight) {
        final double width = FieldUtil.scaleInchesToPixel(2, canvasWidth, canvasHeight);
        final double pixelLength = FieldUtil.scaleInchesToPixel(length, canvasWidth, canvasHeight);
        Vector2d coords = FieldUtil.fieldCoordsToScreenCoords(this.pose.vec());

        AffineTransform transform = new AffineTransform();
        transform.translate(coords.getX(), coords.getY());
        transform.rotate(Math.atan2(pose.headingVec().getX(), pose.headingVec().getY()));
        transform.scale(width, -pixelLength);

        graphics2D.transform(transform);
        graphics2D.setColor(color);
        graphics2D.fillRect(0, 0, 1, 1);

        try {
            graphics2D.transform(transform.createInverse());
        } catch (NoninvertibleTransformException ignored) {}
    }

    public void setLength(double length) {
        this.length = Math.min(length, maxLength);
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void setExtending(boolean extending) {
        this.extending = extending;
    }

    @Override
    public void setCanvasDimensions(double v, double v1) {

    }

    @Override
    public void update(long deltaTime) {
        if(this.extending && this.length < this.maxLength) {
            this.length = Math.min(this.maxLength, this.length + this.speed * deltaTime);
        }
    }
}
