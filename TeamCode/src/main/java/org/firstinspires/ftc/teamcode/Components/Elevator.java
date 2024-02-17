package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import android.os.health.PidHealthStats;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        NOT_RESET
    }
    public State state;

    public static PIDFCoefficients pidCoefficients = new PIDFCoefficients(10, 10, 1, 0);
    private Telemetry telemetry;
    private double position, velocity, previousPosition;
    private Encoder encoder1, encoder2;
    private MotionProfile motionProfile;
    private double ticksToMM = 180*4 / 51000.f;
    private double DistanceToTicks = 51000.f/ (180*4);
    public static double maxAcc = 100, maxVelo = 300;
    public static int error2 = 0;
    public static PIDFCoefficients lastPIDF = null;


    public Elevator(){
        telemetry = ControlHub.telemetry;
        encoder1 = new Encoder(ControlHub.motor[0]);
        encoder2 = new Encoder(ControlHub.motor[1]);
        motionProfile = new MotionProfile(maxVelo*DistanceToTicks, maxAcc*DistanceToTicks);
        ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ControlHub.motor[0].setTargetPosition(0);
        ControlHub.motor[1].setTargetPosition(0);

        ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ControlHub.motor[0].setPower(1);
        ControlHub.motor[1].setPower(1);

        lastPIDF = ControlHub.motor[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        state = State.NOT_RESET;
        resetTime.reset();
    }
    private double targetPos = 0;
    public void setTargetPosition(double p){

        targetPos = p;

        motionProfile.startMotion(this.position, p);
        motionProfile.update();
    }

    public boolean reatchedTargetPosition(){
        return motionProfile.motionEnded();
    }

    private ElapsedTime veloTime = new ElapsedTime(), resetTime = new ElapsedTime();
    private boolean firstUpdate = true;

    @Override
    public void update(){
        if(firstUpdate) {resetTime.reset(); firstUpdate = false;}
        if(pidCoefficients != lastPIDF) {
            ControlHub.motor[0].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            ControlHub.motor[1].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            lastPIDF = pidCoefficients;
        }

        if(state != State.RESET) {
            if (position <= 1 && targetPos <= 0) {
//            ControlHub.motor[0].setPower(0);
//            ControlHub.motor[1].setPower(0);
                ControlHub.motor[0].setMotorDisable();
                ControlHub.motor[1].setMotorDisable();

            } else {
                ControlHub.motor[0].setMotorEnable();
                ControlHub.motor[1].setMotorEnable();
                ControlHub.motor[0].setTargetPosition((int) motionProfile.getPosition());
                ControlHub.motor[1].setTargetPosition((int) motionProfile.getPosition() - error2);
            }
        }

        if(Controls.DownElevator){
            state = State.RESET;
            ControlHub.motor[0].setMotorEnable();
            ControlHub.motor[1].setMotorEnable();

            ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            ControlHub.setMotorPower(MOTOR_PORTS.M0, -0.5);
            ControlHub.setMotorPower(MOTOR_PORTS.M1, -0.5);
        } else if(Controls.ResetElevator){
            state = State.NOT_RESET;
            ControlHub.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ControlHub.motor[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            ControlHub.motor[0].setTargetPosition(0);
            ControlHub.motor[1].setTargetPosition(0);

            ControlHub.motor[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ControlHub.motor[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ControlHub.motor[0].setPower(1);
            ControlHub.motor[1].setPower(1);

        }

    }
    @Override
    public void runTelemetry(){
        telemetry.update();

        telemetry.addLine("\n----Elevator----\n");
        telemetry.addData("power usage",
                ControlHub.getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.AMPS) + ControlHub.getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.AMPS));
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
        velocity = Math.abs(position - previousPosition) / veloTime.seconds();
        previousPosition = position;
        position = encoder1.getCurrentPosition();
        error2 = (int) (position - encoder2.getCurrentPosition());
        motionProfile.update();
        veloTime.reset();
    }
}