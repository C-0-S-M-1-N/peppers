package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M0;
import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M1;
import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M2;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import android.os.health.PidHealthStats;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.opencv.core.Mat;

import java.util.Objects;
import java.util.ResourceBundle;


@Config
public class Elevator implements Part {
    public enum State{
        RESET,
        TO_RESET,
        RUN_DOWN,
        NOT_RESET
    }
    public State state;
    public static PIDFCoefficients positionPIDF = new PIDFCoefficients(7, 0, 0, 0), velocityPIDF = new PIDFCoefficients(5, 5, 0, 0);
    public double targetPosition = 0, livePosition = 0, position_threshold = 5;
    private PIDFCoefficients lastposPID = new PIDFCoefficients(), lastveloPID = new PIDFCoefficients();

    public static boolean RESET = false;

    public Elevator(){
        ControlHub.setMotorDirection(M0, DcMotorSimple.Direction.REVERSE);
        ControlHub.setMotorDirection(M1, DcMotorSimple.Direction.REVERSE);
        for(int i = 0; i < 3; i++){
            ControlHub.motor[i].setVelocityPIDFCoefficients(velocityPIDF.p, velocityPIDF.i, velocityPIDF.d, velocityPIDF.f);
            ControlHub.motor[i].setPositionPIDFCoefficients(positionPIDF.p);
            if(!RESET) ControlHub.motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ControlHub.motor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ControlHub.motor[i].setTargetPosition(0);
            ControlHub.motor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ControlHub.motor[i].setPower(1);
        }
//        positionPIDF = ControlHub.motor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
//        veloPIDF = ControlHub.motor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        RESET = true;
    }

    public void setInstantPosition(double p) {
        setTargetPosition(p);
    }
    public void setTargetPosition(double p){
        targetPosition = p;
    }

    public boolean reatchedTargetPosition(){
        return Math.abs(livePosition - targetPosition) <= position_threshold;
    }

    public double getLivePosition(){
        return livePosition;
    }

    public static boolean DEBUG = false;

    @Override
    public void update() {
        if(DEBUG){
            for(int i = 0; i < 3; i++){
                ControlHub.motor[i].setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, positionPIDF);
            }
        }
        if(DEBUG){
            for(int i = 0; i < 3; i++){
                ControlHub.motor[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
            }
        }

        if(targetPosition <= 0 && livePosition <= 0 && state != State.RESET){
            state = State.RESET;
            for(int i = 0; i < 3; i++){
                ControlHub.motor[i].setMotorDisable();
            }
        }

        if(targetPosition > 0 && state == State.RESET){
            state = State.NOT_RESET;
            for(int i = 0; i < 3; i++){
                ControlHub.motor[i].setMotorEnable();
            }
        }

        ControlHub.setMotorTargetPosition(M1, (int) targetPosition);
        ControlHub.setMotorTargetPosition(M2, (int)(targetPosition - error1));
        ControlHub.setMotorTargetPosition(M0, (int) (targetPosition - error2));
    }
    @Override
    public void runTelemetry(){
        ControlHub.telemetry.addData("targetPosition", targetPosition);
        ControlHub.telemetry.addData("current positoin", livePosition);
    }
    private double error1 = 0, error2 = 0;
    @Override
    public void update_values(){
        livePosition = ControlHub.motor[1].getCurrentPosition();
        error1 = livePosition - ControlHub.motor[2].getCurrentPosition();
        error2 = livePosition - ControlHub.motor[0].getCurrentPosition();
    }
}