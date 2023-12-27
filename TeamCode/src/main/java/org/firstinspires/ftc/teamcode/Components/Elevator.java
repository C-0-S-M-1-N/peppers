package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.ResourceBundle;

@Config
public class Elevator implements Part {
    public enum STATES{
        GO_UP,
        GO_DOWN,
        IDLE
    }
    public STATES STATE;
    private Telemetry telemetry;
//    private AutoMotor left, right;
//    private static DcMotorEx left, right;
    public static int maxPos = 950, elevatorPos;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.f, 0.f, 0.f);
    public static PIDController pidController = new PIDController(pidCoefficients, 0.1);
    public static double ff1 = 0.07, ff2 = 0;

    public Elevator(Telemetry tele){
        telemetry = tele;
        ControlHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);

        STATE = STATES.IDLE;
    }
    @Override
    public void update(){

        switch (STATE){
            case GO_UP:
                break;
            case GO_DOWN:
                break;
        }

        double power = pidController.calculate(-ControlHub.getMotorPosition(ENCODER_PORTS.E0), elevatorPos);

        ControlHub.setMotorPower(MOTOR_PORTS.M0, ff1 + ff2*power);
        ControlHub.setMotorPower(MOTOR_PORTS.M1, ff1 + ff2*power);

        if(-ControlHub.getMotorPosition(ENCODER_PORTS.E0) <= 0.1) STATE = STATES.IDLE;
    }
    public void setPosition(int pos){
        if(pos > elevatorPos) STATE = STATES.GO_UP;
        if(pos < elevatorPos) STATE = STATES.GO_DOWN;
        else STATE = STATES.IDLE;
        elevatorPos = pos;
    }

    @Override
    public void update_values(){
        // nothing to do here :)
    }

    @Override
    public void runTelemetry(){
        telemetry.addData("current position", -ControlHub.getMotorPosition(ENCODER_PORTS.E0));
        telemetry.addData("STATE", STATE.toString());
    }
}