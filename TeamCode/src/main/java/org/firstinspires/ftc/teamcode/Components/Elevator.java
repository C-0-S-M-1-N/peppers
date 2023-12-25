package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.utils.AutoMotor;
import org.firstinspires.ftc.teamcode.utils.PIDController;

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
    private static DcMotorEx left, right;
    private static int maxPos = 950, elevatorPos;
    private double currentPosition = 0, currentVelocity = 0;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.f, 0.f, 0.f);
    public static PIDController pidController;
    public static double ff1 = 0, ff2 = 0;

    public Elevator(HardwareMap hm, Telemetry tele){
        telemetry = tele;
        pidController = new PIDController(pidCoefficients);

        left = hm.get(DcMotorEx.class, "leftElevator");
        right = hm.get(DcMotorEx.class, "rightElevator");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorConfigurationType mct = left.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1);
        left.setMotorType(mct);
        mct = right.getMotorType().clone();
        right.setMotorType(mct);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setTargetPosition(0);
        right.setTargetPosition(0);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        STATE = STATES.IDLE;


//        left.resetZeroPosition();
//        right.resetZeroPosition();

    }
    @Override
    public void update(){
        currentPosition = left.getCurrentPosition();
        currentVelocity = left.getVelocity();

        switch (STATE){
            case GO_UP:
                break;
            case GO_DOWN:
                break;
        }

        double power = pidController.calculate(currentPosition, elevatorPos);
        left.setPower(power);
        right.setPower(power);

        if(currentVelocity <= 0.01) STATE = STATES.IDLE;
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
        telemetry.addData("position", elevatorPos);
        telemetry.addData("STATE", STATE.toString());
    }
}