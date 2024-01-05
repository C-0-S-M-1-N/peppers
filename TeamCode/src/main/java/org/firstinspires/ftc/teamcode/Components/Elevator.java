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
    public static int elevatorPos, currentPosition;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.05*0.9, 0.16*0.9, 0.00092*0.9);
    public static PIDController pidController = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
    public static double ff1 = 0.09, ff2 = 1;
    private static double prevPos = 0, velocity = 0;
    public Elevator(Telemetry tele){
        telemetry = tele;
        ControlHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);
        ControlHub.resetEncoder(ENCODER_PORTS.E0);
        ControlHub.setEncoderDirection(ENCODER_PORTS.E0, Encoder.Direction.REVERSE);

        STATE = STATES.IDLE;
        STATES.wasReseted = true;
    }
    @Override
    public void update(){
        update_values();
        switch (STATE){
            case RESET:
                if(velocity <= 2) {
                    STATE = STATES.IDLE;
                    ControlHub.setMotorPower(MOTOR_PORTS.M0, 0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M1, 0);
                    ControlHub.resetEncoder(ENCODER_PORTS.E0);
                    STATES.wasReseted = true;
                }
                break;
            default:
                double power = pidController.calculate(currentPosition);

                ControlHub.setMotorPower(MOTOR_PORTS.M0, ff1 + ff2*power);
                ControlHub.setMotorPower(MOTOR_PORTS.M1, ff1 + ff2*power);

                if(elevatorPos <= currentPosition + 1 && elevatorPos >= currentPosition - 1) STATE = STATES.IDLE;
                break;

        }
    }
    public void setPosition(int pos){
        if(pos != 0) STATES.wasReseted = false;
        if(pos > elevatorPos) {STATE = STATES.GO_UP;}
        if(pos < elevatorPos) {STATE = STATES.GO_DOWN;}
        else STATE = STATES.IDLE;
        elevatorPos = pos;
        pidController.setSetPoint(pos);
    }

    @Override
    public void update_values(){
        prevPos = currentPosition;
        // nothing to do here :)
        currentPosition = (int) ControlHub.getEncoderPosition(ENCODER_PORTS.E0);
        velocity = Math.abs(currentPosition - prevPos);
    }

    @Override
    public void runTelemetry(){
        telemetry.addData("current position", ControlHub.getEncoderPosition(ENCODER_PORTS.E0));
        telemetry.addData("STATE", STATE.toString());
        telemetry.addData("velocity", velocity);
        telemetry.addData("wasReseted", STATES.wasReseted);
    }
}