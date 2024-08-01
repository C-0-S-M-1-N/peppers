package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M0;
import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M1;
import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M2;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;


@Config
public class Elevator implements Part {
    public enum State{
        RESET,
        TO_RESET,
        RUN_DOWN,
        NOT_RESET
    }
    public State state;
    public double targetPosition = 0, livePosition = 0, position_threshold = 5;
    public static boolean RESET = false;

    public Elevator(){
        ControlHub.setMotorDirection(M2, DcMotorSimple.Direction.REVERSE);
        for(int i = 0; i < 3; i++){
            if(!RESET) ControlHub.motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ControlHub.motor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ControlHub.motor[i].setTargetPosition(ControlHub.motor[i].getCurrentPosition());
            ControlHub.motor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ControlHub.motor[i].setPower(1);
        }
        RESET = true;
        isEnabledCache = true;
    }

    public void setInstantPosition(double p) {
        setTargetPosition(p);
    }
    public void setTargetPosition(double p){
        if(targetPosition != p)
            targetPosition = p;
    }

    public boolean reatchedTargetPosition(){
        return Math.abs(livePosition - targetPosition) <= position_threshold || disableMotors;
    }

    public static int[] PixelLayer = {280, 400, 535, 670, 790, 940, 1100, 1220, 1350};
    private int level = 0;
    public void setLevel(int lvl){
        lvl --;
        if(lvl < 0) lvl = 0;
        if(lvl > 8) lvl = 8;
        setTargetPosition(PixelLayer[lvl]);
        level = lvl;
    }
    public int getLevel(){
        return level;
    }
    public void lvlPlus(){
        setLevel(getLevel() + 1);
    }
    public static int getPositionByLevel(int lvl){
        if(lvl > 8) lvl = 8;
        if(lvl < 0) lvl = 0;
        return PixelLayer[lvl];
    }
    public void lvlMinus(){
        setLevel(getLevel() - 1);
    }
    public double getLivePosition(){
        return livePosition;
    }

    public static boolean DEBUG = false;
    public static boolean disableMotors = false;

    private void disable(){
        if(!isEnabledCache) return;
        for(int i = 0; i < 3; i++){
            ControlHub.motor[i].setMotorDisable();
        }
        isEnabledCache = false;
    }
    private boolean isEnabledCache = false;
    private void enable(){
        if(isEnabledCache) return;
        for(int i = 0; i < 3; i++){
            ControlHub.motor[i].setMotorEnable();
        }
        isEnabledCache = true;
    }

    @Override
    public void update() {
        disableMotors = targetPosition <= 0 && getVelocity() <= 50 && livePosition <= 10;
        /*if(targetPosition <= 0 && livePosition <= 8 && state != State.RESET){
            state = State.RESET;
            for(int i = 0; i < 3; i++){
                ControlHub.motor[i].setMotorDisable();
            }
        }

        if((targetPosition > 0 || livePosition > 8) && state == State.RESET){
            state = State.NOT_RESET;
            for(int i = 0; i < 3; i++){
                ControlHub.motor[i].setMotorEnable();
            }
        }*/
        if(disableMotors) disable();
        else enable();

        ControlHub.setMotorTargetPosition(M2, (int) targetPosition);
        ControlHub.setMotorTargetPosition(M1, (int)(targetPosition - error1));
        ControlHub.setMotorTargetPosition(M0, (int) (targetPosition - error2));
    }
    public double getVelocity(){ return velocity; }
    @Override
    public void runTelemetry() {
        ControlHub.telemetry.addData("targetPosition", targetPosition);
        ControlHub.telemetry.addData("e0", ControlHub.motor[0].getCurrentPosition());
        ControlHub.telemetry.addData("e1", ControlHub.motor[1].getCurrentPosition());
        ControlHub.telemetry.addData("e2", ControlHub.motor[2].getCurrentPosition());
        ControlHub.telemetry.addData("velocity", getVelocity());
    }
    private double error1 = 0, error2 = 0, velocity;
    @Override
    public void update_values(){
        livePosition = ControlHub.motor[2].getCurrentPosition();
        error1 = livePosition - ControlHub.motor[1].getCurrentPosition();
        error2 = livePosition - ControlHub.motor[0].getCurrentPosition();
        velocity = ControlHub.motor[2].getVelocity();
    }
}