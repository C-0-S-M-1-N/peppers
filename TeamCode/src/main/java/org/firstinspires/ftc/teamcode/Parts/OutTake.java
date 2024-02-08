package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.PixelBed;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;


@Config
public class OutTake implements Part{
    public enum State{
        WAITING_FOR_PIXELS,
        EXTENDING,
        EXTENDED,
        RETRACTING,
        RETRACTED;


        public static final double MAX_EXTEND = 950;
        public static double level = 5, step = MAX_EXTEND/10;
    }
    public static State state = State.WAITING_FOR_PIXELS;
    public static Elevator elevator;
    public static ElevatorArm elevatorArm;
    public static Grippers leftGripper, rightGripper;

    public OutTake(HardwareMap hm){
        state = State.WAITING_FOR_PIXELS;

        elevator = new Elevator();
        elevatorArm = new ElevatorArm();

        leftGripper = new Grippers(
                new AutoServo(SERVO_PORTS.S0, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.MICRO_LEGO),
                hm.get(DigitalChannel.class, "cD0")
        );

        rightGripper = new Grippers(
                new AutoServo(SERVO_PORTS.S1, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.MICRO_LEGO),
                hm.get(DigitalChannel.class, "cD1")
        );
    }

    @Override
    public void update(){
        switch (state){
            case WAITING_FOR_PIXELS:
                leftGripper.update();
                rightGripper.update();
                break;
            case EXTENDING:
                elevator.setTargetPosition(100);
                if(elevator.reatchedTargetPosition()){
                    // TODO: calibrate based on robot-backdrop position
                    elevatorArm.setArmAngle(120);
                    state = State.EXTENDED;
                }
            case EXTENDED:
                elevator.setTargetPosition(State.level * State.step);
                break;
            case RETRACTING:
                elevatorArm.setOrientation(90);
                elevator.setTargetPosition(200);
                if(elevatorArm.reachedTargetTourretPosition()){
                    elevatorArm.setArmAngle(0);
                    elevatorArm.setPivotAngle(0);
                }
                if(elevatorArm.reachedStationary()){
                    state = State.RETRACTED;
                }
                break;
            case RETRACTED:
                elevator.setTargetPosition(0);
                state = State.WAITING_FOR_PIXELS;
                break;
        }
        elevator.update();
        elevatorArm.update();
    }

    @Override
    public void update_values(){
        elevator.update_values();
        elevatorArm.update_values();
        leftGripper.update_values();
        rightGripper.update_values();
    }

    @Override
    public void runTelemetry(){
        elevator.runTelemetry();
        elevatorArm.runTelemetry();
        leftGripper.runTelemetry("LEFT CLAW");
        rightGripper.runTelemetry("RIGHT CLAW");


    }
}
