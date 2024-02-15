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
        IDLE,
        UP,
        DOWN,
        RESET

    }
    public State state = State.IDLE;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.0015, 0.005, 0.00001);
    public static PIDController pidController = new PIDController(pidCoefficients);

    private Telemetry telemetry;
    private final ElapsedTime time = new ElapsedTime();
    private double targetPosition, position, velocity, previousPosition, power;
    private Encoder encoder;
    private MotionProfile motionProfile;
    private double ticksToMM = 180*4 / 51000.f;
    private double DistanceToTicks = 51000.f/ (180*4);
    public static double maxAcc = 6500, maxVelo = 30000;
    public static double ff1 = 0;

    public Elevator(){
        telemetry = ControlHub.telemetry;
//        ControlHub.resetEncoder(ENCODER_PORTS.E0);
        encoder = new Encoder(ControlHub.motor[0]);
//        encoder.setDirection(Encoder.Direction.REVERSE);
        motionProfile = new MotionProfile(maxVelo*DistanceToTicks, maxAcc*DistanceToTicks);
//        state = State.RESET;
    }

    public void setTargetPosition(double position){
        if(targetPosition > position) state = State.UP;
        else if(targetPosition < position) state = State.DOWN;
        else state = State.IDLE;

        targetPosition = position;
        pidController.setTargetPosition(targetPosition);
        motionProfile.startMotion(this.position, targetPosition);
    }

    public boolean reatchedTargetPosition(){
        return state == State.IDLE;
    }

    @Override
    public void update(){
        if (Objects.requireNonNull(state) == State.RESET) {
            ControlHub.setMotorPower(MOTOR_PORTS.M0, -1);
            ControlHub.setMotorPower(MOTOR_PORTS.M1, -1);
            if (velocity <= 200 && time.seconds() >= 0.2) {
                state = State.IDLE;
                ControlHub.setMotorPower(MOTOR_PORTS.M0, 0);
                ControlHub.setMotorPower(MOTOR_PORTS.M1, 0);
//                ControlHub.resetEncoder(ENCODER_PORTS.E0);
                encoder = new Encoder(ControlHub.motor[0]);

            }
        } else {
            if(!motionProfile.motionEnded()){
                pidController.clamp = 0;
            } else pidController.clamp = 1;
            if(motionProfile.motionEnded() && Math.abs(targetPosition - position) <= 200) pidController.clamp = 0;
            else pidController.clamp = 1;

            pidController.setTargetPosition(motionProfile.getPosition());
            power = pidController.calculatePower(position);

//            if (targetPosition == 0) {
//                power = 0;
//            }

            if(motionProfile.motionEnded()) state = State.IDLE;

            ControlHub.setMotorPower(MOTOR_PORTS.M0, ff1 + power);
            ControlHub.setMotorPower(MOTOR_PORTS.M1, ff1 + power);
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
        telemetry.addData("motion profile position", motionProfile.getPosition());
        telemetry.addData("current position", position);
        telemetry.addData("velocity", velocity);

    }
    @Override
    public void update_values(){
        velocity = Math.abs(position - previousPosition);
        previousPosition = position;
        position = encoder.getCurrentPosition();
        motionProfile.update();
    }
}