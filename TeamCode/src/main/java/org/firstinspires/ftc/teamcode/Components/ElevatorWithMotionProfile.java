package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.Encoder;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

@Config
public class ElevatorWithMotionProfile {
    public enum STATES{
        GO_UP,
        GO_DOWN,
        RESET,
        IDLE;
    }
    public STATES STATE;
    private Telemetry telemetry;
    public int elevatorPos, currentPosition;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);
    public static PIDController pidController = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

    public ElevatorWithMotionProfile(Telemetry tele){
        telemetry = tele;
        ControlHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);
        ControlHub.resetEncoder(ENCODER_PORTS.E0);
        ControlHub.setEncoderDirection(ENCODER_PORTS.E0, Encoder.Direction.REVERSE);

        STATE = STATES.IDLE;
    }

    public void update(){
        update_values();
        double power = pidController.calculate(currentPosition);
        ControlHub.setMotorPower(MOTOR_PORTS.M0, power);
        ControlHub.setMotorPower(MOTOR_PORTS.M1, power);

        if(elevatorPos == currentPosition) STATE = STATES.IDLE;

    }
    public void setPosition(int pos){
        if(pos != 0) Elevator.STATES.wasReseted = false;
        if(pos > elevatorPos) {STATE = ElevatorWithMotionProfile.STATES.GO_UP;}
        if(pos < elevatorPos) {STATE = ElevatorWithMotionProfile.STATES.GO_DOWN;}
        else STATE = ElevatorWithMotionProfile.STATES.IDLE;
        elevatorPos = pos;
        pidController.setSetPoint(pos);
    }
    public void update_values(){
        pidController.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
        // nothing to do here :)
        currentPosition = (int) ControlHub.getEncoderPosition(ENCODER_PORTS.E0);
    }
    public void runTelemetry(){
        telemetry.addData("current position", ControlHub.getEncoderPosition(ENCODER_PORTS.E0));
        telemetry.addData("STATE", STATE.toString());
        telemetry.addData("wasReseted", Elevator.STATES.wasReseted);
    }
}
