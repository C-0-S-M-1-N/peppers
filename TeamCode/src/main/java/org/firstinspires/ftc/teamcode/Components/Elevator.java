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
    public static PIDFCoefficients pidCoefficients = new PIDFCoefficients(10, 10, 1, 0);
    private Telemetry telemetry;
    private double position, velocity, previousPosition;
    private Encoder encoder;
    private MotionProfile motionProfile;
    private double ticksToMM = 180*4 / 51000.f;
    private double DistanceToTicks = 51000.f/ (180*4);
    public static double maxAcc = 100, maxVelo = 300;
    public static PIDFCoefficients lastPIDF = null;

    public Elevator(){
        telemetry = ControlHub.telemetry;
        encoder = new Encoder(ControlHub.motor[0]);
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
    }

    public void setTargetPosition(double position){
        motionProfile.startMotion(this.position, position);
    }

    public boolean reatchedTargetPosition(){
        return motionProfile.motionEnded();
    }

    private ElapsedTime veloTime = new ElapsedTime();

    @Override
    public void update(){
        if(pidCoefficients != lastPIDF) {
            ControlHub.motor[0].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            ControlHub.motor[1].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            lastPIDF = pidCoefficients;
        }

        ControlHub.motor[0].setTargetPosition((int)motionProfile.getPosition());
        ControlHub.motor[1].setTargetPosition((int)motionProfile.getPosition());
    }
    @Override
    public void runTelemetry(){
        telemetry.update();

        telemetry.addLine("\n----Elevator----\n");
        telemetry.addData("power usage",
                ControlHub.getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.AMPS) + ControlHub.getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.AMPS));
        telemetry.addData("motion profile position", motionProfile.getPosition());
        telemetry.addData("current position", position);
        telemetry.addData("velocity", velocity);
        telemetry.addData("p", lastPIDF.p);
        telemetry.addData("i", lastPIDF.i);
        telemetry.addData("d", lastPIDF.d);

    }
    @Override
    public void update_values(){
        velocity = Math.abs(position - previousPosition) / veloTime.seconds();
        previousPosition = position;
        position = encoder.getCurrentPosition();
        motionProfile.update();
        veloTime.reset();
    }
}