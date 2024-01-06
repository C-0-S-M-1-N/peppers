package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoSensor;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.Objects;

@Config
public class Grippers implements Part {
    public boolean Disable = false;
    public enum STATES{
        CLOSED,
        OPEN
    }
    public STATES STATE;

    private AutoServo claw;
    private DigitalChannel sensor;
    public static double closeClaw = 83;
    private Telemetry telemetry;

    public Grippers(AutoServo p, DigitalChannel sens, Telemetry tele){
        telemetry = tele;
        claw = p;
        STATE = STATES.OPEN;
        sensor = sens;
        sensor.setMode(DigitalChannel.Mode.INPUT);
        p.update();
    }

    @Override
    public void update(){
        if(Disable) return;

        if (!sensor.getState() && STATE == STATES.OPEN) {
            STATE = STATES.CLOSED;
            claw.setAngle(closeClaw);
        }
        claw.update();

    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){
        telemetry.addData("sensor state", sensor.getState());
        telemetry.addData("gripper state", STATE.toString());
    }
    public void drop(){
        claw.setAngle(0);
        STATE = STATES.OPEN;
    }


}
