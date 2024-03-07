package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

@Config
public class AprilTagMath {
    public static double distanceToCenter = 18.6 / 2.54;
    public static double AprilTagToINCHES = 25.4; // assume distance is in meters
    public static double[] TAG_X_OFFSET = {0, 0, 0, 0, 31, 25, 19};
    public static double[] TAG_Y_OFFSET = {0, 0, 0, 0, -43, -43, -43};

    public static Pose2d poseFromTag(Pose2d robotPose, AprilTagDetection detection, int id) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        double tagYaw = rot.secondAngle;
        double tagX = detection.pose.z * AprilTagToINCHES;
        double tagY = detection.pose.x * AprilTagToINCHES;

        double robotHeading = robotPose.getHeading();

        double alpha = robotHeading;

        double x_displacement = cos(alpha) * tagX - sin(alpha) * tagY;
        double y_displacement = cos(alpha) * tagY + sin(alpha) * tagX;

        double x_to_center = cos(robotHeading) * distanceToCenter;
        double y_to_center = sin(robotHeading) * distanceToCenter;

        return new Pose2d(-(x_displacement + x_to_center) + TAG_X_OFFSET[id], -(y_displacement + y_to_center) + TAG_Y_OFFSET[id], alpha);
    }
}
