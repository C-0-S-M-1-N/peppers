package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.LowPassFilter;

@Config
public class Grippers implements Part {
    public static boolean manualMode;
    public enum State{
        OPEN,
        CLOSE
    }
    public State state;
    private ColorRangeSensor sensor;
    private AutoServo servo;
    private final ElapsedTime time = new ElapsedTime();
    public double closed_offset = 0;

    public Grippers(AutoServo servo, DigitalChannel sensor){
        this.servo = servo;

        sensor.setMode(DigitalChannel.Mode.INPUT);
        state = State.OPEN;
        manualMode = false;
        update();
        update_values();
    }
    public Grippers(AutoServo servo, ColorRangeSensor sensor){
        this.servo = servo;
        this.sensor = sensor;
        state = State.OPEN;
        manualMode = false;
        update();
        update_values();
    }
    public void open() {
        if(manualMode){
            state = State.OPEN;
        }
    }
    public void close(){
        if(manualMode){
            state = State.CLOSE;
        }
    }
    private void manualUpdate(){
        switch (state){
            case OPEN:
                servo.setAngle(0);
                break;
            case CLOSE:
                servo.setAngle(80 + closed_offset);
                break;
        }
        servo.update();
    }
    private double dist;
    @Override
    public void update(){
        if(manualMode) manualUpdate();
        switch (state){
            case OPEN:
                servo.setAngle(0);
                break;
            case CLOSE:
                if(time.seconds() >= 0.6 && dist > trashHoldDist){
                    state = State.OPEN;
                } else {
                    if(dist <= trashHoldDist) time.reset();
                    servo.setAngle(80 + closed_offset);
                }
                break;
        }
    }
    private ElapsedTime gripperTime = new ElapsedTime();
    public static double trashHoldDist = 10;
    @Override
    public void update_values(){
        if(manualMode) return;
        dist = sensor.getDistance(DistanceUnit.MM);
        if(dist <= trashHoldDist){
            state = State.CLOSE;
        }

        servo.update();
    }

    public void drop(){
        state = State.OPEN;
        update();
    }

    @Override
    public void runTelemetry(){ }

    public void runTelemetry(String s){
        ControlHub.telemetry.addLine("\n----");
        ControlHub.telemetry.addLine(s);

        ControlHub.telemetry.addData("\tState", state.toString());
        ControlHub.telemetry.addData("sensor reading", dist);
        ControlHub.telemetry.addLine("----");
    }

}
