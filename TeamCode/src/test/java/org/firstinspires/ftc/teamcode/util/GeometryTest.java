package org.firstinspires.ftc.teamcode.util;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

class GeometryTest {

    @Test
    void testAngleWrap() {

        Assertions.assertEquals(Geometry.angleWrap(365), 5);
        Assertions.assertEquals(Geometry.angleWrap(355), -5);
        Assertions.assertEquals(Geometry.angleWrap(-5), -5);
        Assertions.assertEquals(Geometry.angleWrap(5), 5);
        Assertions.assertEquals(Geometry.angleWrap(-185), 175);
        Assertions.assertEquals(Geometry.angleWrap(185), -175);
        Assertions.assertEquals(Geometry.angleWrap(180), 180);
        Assertions.assertEquals(Geometry.angleWrap(0), 0);
    }

}