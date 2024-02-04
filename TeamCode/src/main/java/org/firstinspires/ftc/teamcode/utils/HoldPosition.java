package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

@Config
public class HoldPosition {
    public static Localizer localizer;
    public static double xPos = 0, yPos = 0, head = 0;
    public Telemetry telemetry;
    public static PIDCoefficients translationalCoeff = new PIDCoefficients(-0.1, 0, -0.002),
                                  headingCoeff = new PIDCoefficients(-1.5, -0.1, -0.002);
    public PIDController translationalPID = new PIDController(translationalCoeff),
                         rotationPID      = new PIDController(headingCoeff);
    public HoldPosition(Telemetry t, HardwareMap hm) {
        telemetry = t;
        translationalPID.setTargetPosition(0);
        translationalPID.setMaxActuatorOutput(1);
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        localizer = new StandardTrackingWheelLocalizer(hm, lastTrackingEncPositions, lastTrackingEncVels);
    }

    public void holdPos(double x, double y, double heading){
        xPos = x;
        yPos = y;
        head = heading;
    }

    public double[] update(){
        localizer.update();

        Pose2d pose = localizer.getPoseEstimate();

        double xerr = pose.getX() - xPos,
                yerr = pose.getY() - yPos;

        double xRot = xerr * Math.cos(-pose.getHeading()) - yerr * Math.sin(-pose.getHeading());
        double yRot = xerr * Math.sin(-pose.getHeading()) + yerr * Math.cos(-pose.getHeading());

        double module = Math.abs(xerr * xerr - yerr * yerr);
        double translationalPower = translationalPID.calculatePower(module);
        double alpha = pose.getHeading() - head;
        if(alpha > 3.1415926535) alpha -= 2*3.1415926535;
        if(alpha < -3.1415926535) alpha += 2*3.1415926535;
        double headigPower = rotationPID.calculatePower(alpha);

        xRot *= translationalPower / module;
        yRot *= translationalPower / module;

        telemetry.addData("module", module);
        telemetry.addData("rotation", alpha);
        // TODO: add heading

        translationalPID.setPidCoefficients(translationalCoeff);
        rotationPID.setPidCoefficients(headingCoeff);

        return new double[]{-xRot, yRot, headigPower};
    }
    public double getDistFromTarget() {
        Pose2d pose = localizer.getPoseEstimate();
        double xerr = pose.getX() - xPos,
                yerr = pose.getY() - yPos;
        return sqrt(xerr*xerr + yerr*yerr);
    }

}
