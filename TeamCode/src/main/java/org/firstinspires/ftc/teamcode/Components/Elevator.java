package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.Encoder;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.ResourceBundle;


@Config
public class Elevator implements Part {
    public enum STATES{
        GO_UP,
        GO_DOWN,
        RESET,
        IDLE;

    }
    public STATES STATE;
    private Telemetry telemetry;
    public int elevatorPos, currentPosition;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.15, 0, 0.0012);

    public static PIDController pidController = new PIDController(pidCoefficients);
    public static double ff1 = 0, ff2 = 0.08;
    private static double prevPos = 0, velocity = 0;
    public Elevator(Telemetry tele){
        telemetry = tele;
        ControlHub.resetEncoder(ENCODER_PORTS.E0);
        ControlHub.setEncoderDirection(ENCODER_PORTS.E0, Encoder.Direction.REVERSE);

        pidController.setMaxActuatorOutput(1.0);

        STATE = STATES.IDLE;
    }
    @Override
    public void update(){
        update_values();

        switch (STATE){
            case RESET:
                ControlHub.setMotorPower(MOTOR_PORTS.M0, -1);
                ControlHub.setMotorPower(MOTOR_PORTS.M1, -1);
                if(currentPosition <= 0 && Math.abs(ControlHub.getEncoderVelocityReading(ENCODER_PORTS.E0)) <= 0){
                    ControlHub.resetEncoder(ENCODER_PORTS.E0);
                    STATE = STATES.IDLE;
                }
            default:
                double power = pidController.calculatePower(currentPosition);
                ControlHub.setMotorPower(MOTOR_PORTS.M0, power);
                ControlHub.setMotorPower(MOTOR_PORTS.M1, power);
        }

    }
    public void setPosition(int pos){
        if(pos > elevatorPos) {STATE = STATES.GO_UP;}
        else if(pos < elevatorPos) {STATE = STATES.GO_DOWN;}
        else STATE = STATES.IDLE;

        elevatorPos = pos;
        if(elevatorPos <= 0) STATE = STATES.RESET;

        pidController.setTargetPosition(elevatorPos);
    }
    public int getCurrentPosition(){ return (int)currentPosition; }

    @Override
    public void update_values(){
        prevPos = currentPosition;
        pidController.setPidCoefficients(pidCoefficients);
        currentPosition = (int) ControlHub.getEncoderPosition(ENCODER_PORTS.E0);
        velocity = abs(currentPosition - prevPos);
    }

    @Override
    public void runTelemetry(){
        telemetry.addData("current position", ControlHub.getEncoderPosition(ENCODER_PORTS.E0));
        telemetry.addData("STATE", STATE.toString());
        telemetry.addData("velocity", velocity);
        telemetry.addData("Elevator power consumption", ControlHub.getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.MILLIAMPS) +
                ControlHub.getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.MILLIAMPS));
        telemetry.addData("targetPos", elevatorPos);
    }
}