package com.mrcod.meepmeep.entity.field;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.entity.ThemedEntity;
import com.noahbres.meepmeep.core.util.FieldUtil;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;

public class CarouselEntity extends BasicThemedEntity {
    private final Vector2d position;

    public CarouselEntity(MeepMeep meepMeep, Vector2d position) {
        super(meepMeep, "CAROUSEL_FIELD_ENTITY");
        this.position = position;
    }

    @Override
    public void render(@NotNull Graphics2D graphics2D, int canvasWidth, int canvasHeight) {
        final double size = FieldUtil.scaleInchesToPixel(7.5, canvasWidth, canvasHeight);
        final double halfSize = size / 2;

        AffineTransform transform = new AffineTransform();
        Vector2d coords = FieldUtil.fieldCoordsToScreenCoords(this.position);
        transform.translate(coords.getX() - halfSize, coords.getY() - halfSize);
        transform.scale(size, size);

        graphics2D.transform(transform);
        graphics2D.setColor(new Color(0, 0, 0));
        graphics2D.drawOval(0, 0, 1, 1);
        try {
            graphics2D.transform(transform.createInverse());
        } catch (NoninvertibleTransformException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void setCanvasDimensions(double v, double v1) {

    }

    @Override
    public void update(long l) {

    }

    @Override
    public void switchScheme(@NotNull ColorScheme colorScheme) {

    }
}
