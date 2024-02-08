package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import android.os.health.PidHealthStats;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.opencv.core.Mat;

import java.util.Objects;


@Config
public class Elevator implements Part {
    public enum State{
        IDLE,
        UP,
        DOWN,
        RESET

    }
    public State state = State.IDLE;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);
    public static PIDController pidController = new PIDController(pidCoefficients);

    private Telemetry telemetry;
    private final ElapsedTime time = new ElapsedTime();
    private double targetPosition, position, velocity, previousPosition, power;

    public Elevator(){
        telemetry = ControlHub.telemetry;
        ControlHub.resetEncoder(ENCODER_PORTS.E0);

        state = State.RESET;
    }

    public void setTargetPosition(double position){
        if(targetPosition > position) state = State.UP;
        else if(targetPosition < position) state = State.DOWN;
        else state = State.IDLE;

        targetPosition = position;
        if(targetPosition == 0){
            state = State.RESET;
            time.reset();
        }
        pidController.setTargetPosition(targetPosition);
    }

    public boolean reatchedTargetPosition(){
        return state == State.IDLE;
    }

    @Override
    public void update(){
        if (Objects.requireNonNull(state) == State.RESET) {
            ControlHub.setMotorPower(MOTOR_PORTS.M0, -1);
            ControlHub.setMotorPower(MOTOR_PORTS.M1, -1);
            if (velocity == 0 && time.seconds() >= 0.5) {
                state = State.IDLE;
                ControlHub.setMotorPower(MOTOR_PORTS.M0, 0);
                ControlHub.setMotorPower(MOTOR_PORTS.M1, 0);
                ControlHub.resetEncoder(ENCODER_PORTS.E0);

            }
        } else {
            double power = pidController.calculatePower(position);

            if (targetPosition == 0) {
                power = 0;
            }

            if(Math.abs(targetPosition - position) <= 2) state = State.IDLE;

            ControlHub.setMotorPower(MOTOR_PORTS.M0, power);
            ControlHub.setMotorPower(MOTOR_PORTS.M1, power);
        }
    }
    @Override
    public void runTelemetry(){
        telemetry.update();

        telemetry.addLine("\n----Elevator----\n");
        telemetry.addData("power usage",
                ControlHub.getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.AMPS) + ControlHub.getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.AMPS));
        telemetry.addData("elevator power", power);
        telemetry.addData("target position", targetPosition);
        telemetry.addData("current position", position);
        telemetry.addData("velocity", velocity);

        telemetry.update();
    }
    @Override
    public void update_values(){
        velocity = Math.abs(position - previousPosition);
        previousPosition = position;
        position = ControlHub.getEncoderPosition(ENCODER_PORTS.E0);
    }
}