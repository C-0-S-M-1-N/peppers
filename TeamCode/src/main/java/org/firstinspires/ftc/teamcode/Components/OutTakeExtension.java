package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.LowPassFilter;
import org.opencv.core.Mat;

@Config
public class OutTakeExtension implements Part {
    private DistanceSensor sensor;
    private AutoServo servo;
    public static double t = 0.5;
    public static LowPassFilter filter = new LowPassFilter(t);
    public static double length, angle;
    private Telemetry telemetry = ControlHub.telemetry;
    public static boolean active = false;
    public static double armLenghtInMM = 210;

    public static double Start = 80;
    public static double End = 20;

    public OutTakeExtension(DistanceSensor sensor, AutoServo servo){

        this.sensor = sensor;
        this.servo = servo;
    }

    public void activate(){
        active = true;
    }
    public void deactivate(){
        active = false;
    }
    public void setImuAngle(double angle){
        if(angle < 0) angle += 2 * Math.PI;
        this.angle = angle;
    }
    private static final double t_min = 66.5, a = 29.8, b = 65, c = 134.387;
    private double getServoAngleByLenght(double l){
        if(l > 280) return 0;
        if(Double.isNaN(l)) return 0;

        double T = l / 120.0;
        l += Start * (1 - T) + End * T;

        l += t_min;

        double theta = - (c*c - a*a - l*l - b*b) / (2*b*Math.sqrt(a*a + l*l));
        if(Math.abs(theta) > 1) theta = 1 * Math.signum(theta);
        theta = Math.acos(theta);
        theta = Math.toDegrees(theta);

        double sAngle = l / Math.sqrt(a * a + l * l);
        if(Math.abs(sAngle) > 1) sAngle = 1 * Math.signum(sAngle);
        sAngle = Math.asin(sAngle);
        if(sAngle < 0) sAngle += Math.PI / 2;

        return 220 - theta - Math.toDegrees(sAngle);
    }
    private double lastA = 0;
    @Override
    public void update(){
        double a = Math.min(getServoAngleByLenght(length), 118);
        if(Double.isNaN(a)) a = lastA;
        if(!active) a = 0;

        servo.setAngle(a);
        servo.update();
        lastA = a;

        if(Controls.DownElevator) sensor.resetDeviceConfigurationForOpMode();
    }
    @Override
    public void update_values(){
        angle = ExpansionHub.ImuYawAngle;
        angle = Math.toRadians(angle);
        if(angle < 0) angle += 2 * Math.PI;
        filter.setT(t);
        length = filter.pass(ExpansionHub.sensorDistance) - armLenghtInMM / Math.cos(angle);
    }

    @Override
    public void runTelemetry(){
        telemetry.addLine("\n------- EXTENSION --------");
        telemetry.addData("recorded length", length);
        telemetry.addData("raw servo angle", getServoAngleByLenght(length));
    }
}
