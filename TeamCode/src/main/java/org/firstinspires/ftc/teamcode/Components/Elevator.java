package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.Encoder;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.utils.PIDController;


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
    public int elevatorPos, currentPosition;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.2, 0.04, 0.0001);

    public static PIDController pidController = new PIDController(pidCoefficients);
    public static double ff1 = 0.18, ff2 = 0.08;
    public static boolean RETRACTING = false;
    private static double prevPos = 0, velocity = 0;
    public Elevator(Telemetry tele){
        telemetry = tele;
//        ControlHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);
        ControlHub.resetEncoder(ENCODER_PORTS.E0);
        ControlHub.setEncoderDirection(ENCODER_PORTS.E0, Encoder.Direction.REVERSE);

        pidController.setMaxActuatorOutput(1.0);

        STATE = STATES.IDLE;
        STATES.wasReseted = true;
    }
    @Override
    public void update(){
        update_values();
        telemetry.addData("error", elevatorPos - currentPosition);

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
                if(elevatorPos == 0 && RETRACTING && currentPosition < 5) RETRACTING = false;

                double power = pidController.calculatePower(currentPosition);

                telemetry.addData("power: ", power);

                ControlHub.setMotorPower(MOTOR_PORTS.M0, ff1 + ff2*power);
                ControlHub.setMotorPower(MOTOR_PORTS.M1, ff1 + ff2*power);

                if(elevatorPos <= currentPosition + 3 && elevatorPos >= currentPosition - 3) STATE = STATES.IDLE;
                break;

        }
    }
    public void setPosition(int pos){
        if(pos != 0) STATES.wasReseted = false;
        if(pos > elevatorPos) {STATE = STATES.GO_UP;}
        else if(pos < elevatorPos) {STATE = STATES.GO_DOWN;}
        else STATE = STATES.IDLE;
        elevatorPos = pos;
        pidController.setTargetPosition(elevatorPos);
    }
    public int getCurrentPosition(){ return (int)currentPosition; }

    @Override
    public void update_values(){
        prevPos = currentPosition;
        pidController.setPidCoefficients(pidCoefficients);
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