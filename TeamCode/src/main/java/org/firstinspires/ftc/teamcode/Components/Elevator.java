package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.Encoder;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;


@Config
public class Elevator implements Part {
    public enum STATES{
        GO_UP,
        GO_DOWN,
        RESET,
        IDLE;

        public static boolean wasReseted = true;
    }
    public STATES STATE;
    private Telemetry telemetry;
//    private AutoMotor left, right;
//    private static DcMotorEx left, right;
    public static int elevatorPos;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.005, 0.02, 0.0005);
    public static PIDController pidController = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
    public Elevator(Telemetry tele){
        telemetry = tele;
        ControlHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);
        ControlHub.resetEncoder(ENCODER_PORTS.E0);
        ControlHub.setEncoderDirection(ENCODER_PORTS.E0, Encoder.Direction.FORWARD);

        STATE = STATES.IDLE;
        STATES.wasReseted = true;
    }
    @Override
    public void update(){
        pidController.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

        switch (STATE){
            case RESET:
                if(ControlHub.getMotorVelocity(ENCODER_PORTS.E0) <= 0.01) {
                    STATE = STATES.IDLE;
                    ControlHub.setMotorPower(MOTOR_PORTS.M0, 0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M1, 0);
                    STATES.wasReseted = true;
                }
                break;
            default:
                double power = pidController.calculate(ControlHub.getEncoderPosition(ENCODER_PORTS.E0));

                ControlHub.setMotorPower(MOTOR_PORTS.M0, power);
                ControlHub.setMotorPower(MOTOR_PORTS.M1, power);

                if (ControlHub.getMotorVelocity(ENCODER_PORTS.E0) <= 0.1 && !STATES.wasReseted){
                    if(elevatorPos == 0){
                        ControlHub.setMotorPower(MOTOR_PORTS.M0, -1);
                        ControlHub.setMotorPower(MOTOR_PORTS.M1, -1);
                        STATE = STATES.RESET;
                    }
                }
                break;

        }
    }
    public void setPosition(int pos){
        if(pos > elevatorPos) {STATE = STATES.GO_UP; STATES.wasReseted = false;}
        if(pos < elevatorPos) {STATE = STATES.GO_DOWN; STATES.wasReseted = false;}
        else STATE = STATES.IDLE;
        elevatorPos = pos;
        pidController.setSetPoint(pos);
    }

    @Override
    public void update_values(){
        // nothing to do here :)
    }

    @Override
    public void runTelemetry(){
        telemetry.addData("current position", ControlHub.getEncoderPosition(ENCODER_PORTS.E0));
        telemetry.addData("STATE", STATE.toString());
    }
}