package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M0;
import static org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS.M1;
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

    public static PIDFCoefficients pidCoefficients = new PIDFCoefficients(10, 10, 1, 5);
    private final Telemetry telemetry;
    private double position, velocity;
    private final MotionProfile motionProfile;
    public static double maxAcc = 60, maxVelo = 200;
    public static int error2 = 0;
    public static PIDFCoefficients lastPIDF = null;
    private final ElapsedTime resetElevator = new ElapsedTime();


    public Elevator(){
        telemetry = ControlHub.telemetry;
        double distanceToTicks = 51000.f / (180 * 4);
        motionProfile = new MotionProfile(maxVelo* distanceToTicks, maxAcc* distanceToTicks);

        ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ControlHub.motor[0].setTargetPosition(-60);
        ControlHub.motor[1].setTargetPosition(-60);

        ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ControlHub.motor[0].setPower(1);
        ControlHub.motor[1].setPower(1);

        lastPIDF = ControlHub.motor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        state = State.NOT_RESET;
        resetTime.reset();
    }
    private double targetPos = 0;

    public void setInstantPosition(double p) {
        targetPos = p;

        motionProfile.startMotion(p-0.1, p);
        motionProfile.update();
    }
    public void setTargetPosition(double p){

        targetPos = p;

        motionProfile.startMotion(position, targetPos);
        motionProfile.update();

        if(p < 0) state = State.RESET;
    }

    public boolean reatchedTargetPosition(){
        return motionProfile.motionEnded();
    }

    private ElapsedTime resetTime = new ElapsedTime();
    private boolean firstUpdate = true;
    public double getLivePosition(){
        return position;
    }


    private boolean MOTOR_ENABLED = true;
    @Override
    public void update() {
        if(firstUpdate) {resetTime.reset(); firstUpdate = false;}
        if(pidCoefficients != lastPIDF) {
            ControlHub.motor[0].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            ControlHub.motor[1].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            lastPIDF = pidCoefficients;
        }

        if(state == State.NOT_RESET) {
            if (position <= 23  && targetPos <= 0) {
                if(MOTOR_ENABLED) {
                    ControlHub.motor[0].setMotorDisable();
                    ControlHub.motor[1].setMotorDisable();
                    MOTOR_ENABLED = false;
                }
            } else {
                if(!MOTOR_ENABLED) {
                    ControlHub.motor[0].setMotorEnable();
                    ControlHub.motor[1].setMotorEnable();
                    MOTOR_ENABLED = true;
                }

                ControlHub.setMotorTargetPosition(M0, (int) motionProfile.getPosition() + 20);
                ControlHub.setMotorTargetPosition(M1, (int) motionProfile.getPosition() + 20 - error2);
            }
        }

        if(state == State.RESET) {

            ControlHub.motor[0].setMotorEnable();
            ControlHub.motor[1].setMotorEnable();

            ControlHub.setMotorTargetPosition(M0, -6900);
            ControlHub.setMotorTargetPosition(M1, -6900 - error2);
            resetElevator.reset();
            state = State.RUN_DOWN;
        }

        if(velocity <= 1 && state == State.RUN_DOWN && resetElevator.seconds() > 0.2) {
            ControlHub.motor[0].setMotorDisable();
            ControlHub.motor[1].setMotorDisable();

            state = State.TO_RESET;
            resetTime.reset();
        }

        if(state == State.TO_RESET) {
            ControlHub.motor[0].setMotorEnable();
            ControlHub.motor[1].setMotorEnable();
            MOTOR_ENABLED = true;

            ControlHub.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ControlHub.motor[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            ControlHub.setMotorTargetPosition(M0, -60);
            ControlHub.setMotorTargetPosition(M1, -60);

            ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ControlHub.motor[0].setPower(1);
            ControlHub.motor[1].setPower(1);
            state = State.NOT_RESET;
        }

    }
    @Override
    public void runTelemetry(){

        telemetry.addLine("\n----Elevator----\n");
//        telemetry.addData("power usage",
//                ControlHub.getCurrentFromMotor(M0, CurrentUnit.AMPS) + ControlHub.getCurrentFromMotor(M1, CurrentUnit.AMPS));
        telemetry.addData("motion profile position", motionProfile.getPosition());
        telemetry.addData("velocity", velocity);
        telemetry.addData("position", position);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("ELEVATOR STATE", state.toString());
        telemetry.addData("p", lastPIDF.p);
        telemetry.addData("i", lastPIDF.i);
        telemetry.addData("d", lastPIDF.d);

    }
    @Override
    public void update_values(){
        velocity = ControlHub.motor[0].getVelocity();
        position = ControlHub.motor[0].getCurrentPosition();
        error2 = (int) (position - ControlHub.motor[1].getCurrentPosition());
        motionProfile.update();
    }
}