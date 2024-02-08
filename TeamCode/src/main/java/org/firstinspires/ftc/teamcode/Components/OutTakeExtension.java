package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.LowPassFilter;

@Config
public class OutTakeExtension implements Part {
    private DistanceSensor sensor;
    private AutoServo servo;
    public static double t = 0.2;
    public static LowPassFilter filter = new LowPassFilter(t);
    public static double spoolSizeInMM = 32, sensorOffset = 15;
    private double currentExtendedSize = 0, length, angle;
    private Telemetry telemetry = ControlHub.telemetry;
    public static boolean active = false;

    public OutTakeExtension(DistanceSensor sensor, AutoServo servo){

        this.sensor = sensor;
        this.servo = servo;
    }

    private void setLengthInMM(double len){
        angle = (len * 360) / (2 * Math.PI * spoolSizeInMM);
        servo.setAngle(angle);
    }
    public void reset(){
        currentExtendedSize = 0;
        setLengthInMM(0);
    }
    public void activate(){
        active = true;
    }
    public void deactivate(){
        active = false;
    }

    @Override
    public void update(){
        if(!active) return;
        currentExtendedSize += length - sensorOffset;
        setLengthInMM(currentExtendedSize);

    }

    @Override
    public void update_values(){
        filter.setT(t);
        length = filter.pass(sensor.getDistance(DistanceUnit.MM));
    }

    @Override
    public void runTelemetry(){
        telemetry.addLine("\n------- EXTENSION --------");
        telemetry.addData("recorded length", length);
        telemetry.addData("angle", angle);
    }
}
