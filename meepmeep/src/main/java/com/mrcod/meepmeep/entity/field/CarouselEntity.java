package com.mrcod.meepmeep.entity.field;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.entity.ThemedEntity;
import com.noahbres.meepmeep.core.util.FieldUtil;

import org.jetbrains.annotations.NotNull;

import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;

public class CarouselEntity implements ThemedEntity {
    private final MeepMeep meepMeep;
    private int zIndex = 0;
    private final Vector2d position;

    public CarouselEntity(MeepMeep meepMeep, Vector2d position) {
        this.meepMeep = meepMeep;
        this.position = position;
    }

    @NotNull
    @Override
    public MeepMeep getMeepMeep() {
        return meepMeep;
    }

    @NotNull
    @Override
    public String getTag() {
        return "CAROUSEL_FIELD_ENTITY";
    }

    @Override
    public int getZIndex() {
        return zIndex;
    }

    @Override
    public void setZIndex(int i) {
        this.zIndex = i;
    }

    @Override
    public void render(@NotNull Graphics2D graphics2D, int canvasWidth, int canvasHeight) {
        Vector2d coords = FieldUtil.fieldCoordsToScreenCoords(this.position);

        AffineTransform transform = new AffineTransform();
        transform.translate(coords.getX(), coords.getY());
        transform.scale(FieldUtil.scaleInchesToPixel(15, canvasWidth, canvasHeight),
                FieldUtil.scaleInchesToPixel(15, canvasWidth, canvasHeight));

        graphics2D.transform(transform);
        graphics2D.drawOval(0, 0, 1, 1);
        graphics2D.
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
