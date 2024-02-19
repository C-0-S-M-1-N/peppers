package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

public class AprilTagMath {
    public static Pose2d poseFromTag(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double tagYaw = rot.secondAngle;
        double tagX = detection.pose.z;
        double tagY = detection.pose.x;

        double new_tag_x = cos(tagYaw) * tagX - sin(tagYaw) * tagY;
        double new_tag_y = cos(tagYaw) * tagY + sin(tagYaw) * tagX;

        return new Pose2d(new_tag_x, new_tag_y, tagYaw); // TODO: CHECK SIGNS OF X, Y, YAW
    }
}
