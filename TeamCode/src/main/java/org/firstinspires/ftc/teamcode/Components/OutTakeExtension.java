package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.LowPassFilter;

@Config
public class OutTakeExtension implements Part {
    private DistanceSensor sensor;
    private AutoServo servo;
    public static double t = 0.2;
    public static LowPassFilter filter = new LowPassFilter(t);
    public OutTakeExtension(DistanceSensor sensor, AutoServo servo){

        this.sensor = sensor;
        this.servo = servo;
    }

    @Override
    public void update(){

    }

    @Override
    public void update_values(){
        filter.setT(t);
    }

    @Override
    public void runTelemetry(){

    }
}
