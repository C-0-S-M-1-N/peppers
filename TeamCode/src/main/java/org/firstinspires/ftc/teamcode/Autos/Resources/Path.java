package org.firstinspires.ftc.teamcode.Autos.Resources;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Vector;

public class Path {
    private Pose2d[] points;
    private Vector<Pose2d> path;
    private static final double t = 0.5, resolution = 0.01;

    public Path(Pose2d[] p){
        points = p;
        path = new Vector<Pose2d>();
    }

    public void addPoint(Pose2d p){
        path.clear();
        if(points.length > 2){
            Pose2d P1 = new Pose2d();
            P1 = P1.copy(points[0].getX(), points[0].getY(), points[0].getHeading());
            P1 = P1.times(t).plus( points[1].copy(points[1].getX(), points[1].getY(), points[1].getHeading()).times(1-t));
            for(double T = 0; T <= 1; T += resolution * 2){
                Pose2d P = points[0].copy(points[0].getX(), points[0].getY(), points[0].getHeading()).
                            times(1 - T).plus( P1.copy(P1.getX(), P1.getY(), P1.getHeading()).times(T));
                path.add(P);
            }

            for(int i = 1; i < points.length - 1; i++){
                P1 = points[i-1].copy(points[i-1].getX(), points[i-1].getY(), points[i-1].getHeading()).
                        times(t).plus(points[i].copy(points[i].getX(), points[i].getY(), points[i].getHeading()).times(1-t));
                Pose2d P2 = points[i + 1].copy(points[i+1].getX(), points[i+1].getY(), points[i+1].getHeading()).
                        times(t).plus(points[i].copy(points[i].getX(), points[i].getY(), points[i].getHeading()).times(1-t));

                for(double T = 0; T <= 1; T += resolution){
                }
            }
        }

    }
}
