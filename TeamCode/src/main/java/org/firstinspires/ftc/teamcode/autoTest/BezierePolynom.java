package org.firstinspires.ftc.teamcode.autoTest;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.lang.reflect.Array;
import java.util.Vector;
import java.util.concurrent.atomic.DoubleAccumulator;

public class BezierePolynom {
    private int degree;
    private Vector<Pose2d> points;
    public BezierePolynom(Vector<Pose2d> points){
        degree = points.size() - 1;
        this.points = points;
    }
    public Pose2d evaluate(int t){
        double y = 0, x = 0, rot = 0;
        for(int i = 0; i <= degree; i++) {
            double a = Math.pow(t, i)*Math.pow(1-t, degree - i);
            y += a * points.elementAt(i).getY();
            x += a * points.elementAt(i).getX();
            rot += a * points.elementAt(i).getHeading();
        }
        return new Pose2d(x, y, rot);
    }
}
/**

 a^b * x^y

 a^b' * x^y + a^b*x^y'

 x ^ y * b * a^(b-1)

 * */
