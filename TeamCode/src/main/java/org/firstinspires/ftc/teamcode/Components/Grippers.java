package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class Grippers implements Part {
    public enum State{
        OPEN,
        CLOSE
    }
    public State state;
    private final DigitalChannel sensorGate;
    private final AutoServo servo;
    private final ElapsedTime time = new ElapsedTime();

    public Grippers(AutoServo servo, DigitalChannel sensor){
        this.servo = servo;
        this.sensorGate = sensor;

        sensor.setMode(DigitalChannel.Mode.INPUT);
        state = State.OPEN;
    }

    @Override
    public void update(){
        switch (state){
            case OPEN:
                servo.setAngle(0);
                break;
            case CLOSE:
                if(time.seconds() >= 0.5 && sensorGate.getState()){
                    state = State.OPEN;
                } else {
                    time.reset();
                    servo.setAngle(80);
                }
                break;
        }
    }
    private ElapsedTime gripperTime = new ElapsedTime();
    @Override
    public void update_values(){
        if(!sensorGate.getState()){
            state = State.CLOSE;
            time.reset();
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
        ControlHub.telemetry.addData("beam breaker", sensorGate.getState());
        ControlHub.telemetry.addLine("----");
    }

}
