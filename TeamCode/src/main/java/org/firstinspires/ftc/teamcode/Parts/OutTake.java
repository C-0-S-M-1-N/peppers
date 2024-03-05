package org.firstinspires.ftc.teamcode.Parts;

import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.BetterColorRangeSensor;


@Config
public class OutTake implements Part{

    public enum State{
        WAITING_FOR_PIXELS,
        EXTENDING,
        EXTENDED,
        RETRACTING,
        RETRACTED,
        RELEASING,
        LVL_UP,
        LVL_DOWN,
        NULL;


        public static final double MAX_EXTEND = 925;
        public static double level = 5, step = MAX_EXTEND/10;
    }
    public static State state = State.WAITING_FOR_PIXELS;
    public static Elevator elevator;
    public static ElevatorArm elevatorArm;
    public static Grippers leftGripper, rightGripper;
    public static OutTakeExtension outTakeExtension;
    public boolean align = false;
    public static double finalArmAngle = 210, finalPivotPivotAngle = 130;
    public static double intermediarPivot = 130;
    private ElapsedTime releasingTime = new ElapsedTime();
    private ElapsedTime FUCKING_EXTENSIE = new ElapsedTime();

    public void setElevatorLevel(int level){
        elevator.setTargetPosition(level * State.step);
    }



    public OutTake(HardwareMap hm){
        align = false;
        state = State.WAITING_FOR_PIXELS;

        elevator = new Elevator();
        elevatorArm = new ElevatorArm();
        outTakeExtension = new OutTakeExtension(
                hm.get(DistanceSensor.class, "sensor"),
                new AutoServo(SERVO_PORTS.S3, 0, true, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON));

        leftGripper = new Grippers(
                new AutoServo(SERVO_PORTS.S0, 40.f/180, false, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                hm.get(BetterColorRangeSensor.class, "leftSensor"),
                650
        );

        rightGripper = new Grippers(
                new AutoServo(SERVO_PORTS.S2, 40.f/180, true, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                hm.get(BetterColorRangeSensor.class, "rightSensor"),
                300
        );


        elevatorArm.setArmAngle(0);
        elevatorArm.setPivotAngle(0);

        elevatorArm.update();
        outTakeExtension.deactivate();
        elevator.setTargetPosition(-80);
        state = State.WAITING_FOR_PIXELS;
//        elevator.state = Elevator.State.RESET;
    }
    private void controls(){
        if(Controls.ManualMode) Grippers.manualMode = !Grippers.manualMode;
        if(Controls.ExtendElevator && state != State.NULL) state = State.EXTENDING;
        else if(Controls.RetractElevator) state = State.RETRACTING;
        else {
            if(Controls.ElevatorUp && state == State.NULL) {
                State.level++;
                elevator.setInstantPosition(State.level * State.step);
            } else if(Controls.ElevatorUp) {
                State.level++;
            }
            if(Controls.ElevatorDown && state == State.NULL){
                State.level--;
                elevator.setInstantPosition(State.level * State.step);
            } else if(Controls.ElevatorDown) {
                State.level--;
            }
            if(Controls.DropLeft){
                leftGripper.drop();
            }
            if(Controls.DropRight){
                rightGripper.drop();
            }
        }
        if(Controls.ResetTourret) ExpansionHub.resetIMU();
    }

    boolean extending = false, set0Pos = false;
    private ElapsedTime retractTime = new ElapsedTime();

    @Override
    public void update(){
        controls();
        if(State.level > 10) State.level = 10;
        if(State.level < 0) State.level = 0;
        switch (state){
            case WAITING_FOR_PIXELS:
                if(elevator.state == Elevator.State.NOT_RESET) {
                    leftGripper.update();
                    rightGripper.update();
                }
                break;
            case EXTENDING:

                elevator.setTargetPosition(Math.max(State.step * 2, State.level * State.step));
                state = State.EXTENDED;

                break;
            case EXTENDED:
                if(elevator.getLivePosition() >= State.step * 0.7){
                    elevatorArm.setArmAngle(finalArmAngle);
                    elevatorArm.setPivotAngle(intermediarPivot);
                    elevator.setInstantPosition(State.level * State.step);
                    state = State.NULL;
                }
                break;
            case RELEASING:
                if(releasingTime.time() > 0.2) {
                    outTakeExtension.deactivate();
                }
                if((OutTakeExtension.MOTION_PROFILED && outTakeExtension.getLivePosition() < 80) || (!OutTakeExtension.MOTION_PROFILED && releasingTime.seconds() > 0.5)) {
                    state = State.RETRACTING;
                    align = true;
                }
                break;
            case RETRACTING:
                if(align) {
                    elevator.setTargetPosition(State.step * 1);
                    outTakeExtension.deactivate();
                    elevatorArm.setOrientation(0);
                    elevatorArm.setArmAngle(0);
                    elevatorArm.setPivotAngle(finalPivotPivotAngle / 2 - 15);
                    retractTime.reset();
                }
                align = false;

                if(elevatorArm.reachedStationary() && elevator.reatchedTargetPosition() && retractTime.seconds() >= 0.1){
                    elevatorArm.setPivotAngle(0);
                    state = State.RETRACTED;
                }

                break;
            case RETRACTED:
                if(!set0Pos) {
                    set0Pos = true;
                    elevator.setTargetPosition(-80);
                }
                if(elevator.reatchedTargetPosition()) {
                    state = State.WAITING_FOR_PIXELS;
                    set0Pos = false;
                }
                break;
            case NULL:
                if(elevatorArm.getLiveArmAngle() > 160) {
                    elevatorArm.setPivotAngle(finalPivotPivotAngle);
                    outTakeExtension.activate();
                }
                if(elevatorArm.reachedStationary() && onePixel()) {
                    align = true;
                }

                if(!onePixel() && elevatorArm.reachedStationary()) {
                    state = State.RELEASING;
                    releasingTime.reset();
                }
                break;
        }
        if(align && elevatorArm.reachedStationary()){
            elevatorArm.setOrientation(-ExpansionHub.ImuYawAngle);
        } else {
            elevatorArm.setOrientation(0);
        }

        outTakeExtension.update();
        elevator.update();
        elevatorArm.update();
    }

    @Override
    public void update_values(){
        elevator.update_values();
        elevatorArm.update_values();
        outTakeExtension.update_values();

        Grippers.State leftPrev = leftGripper.state, rightPrev = rightGripper.state;

        leftGripper.update_values();
        rightGripper.update_values();

        if(leftPrev != leftGripper.state){
            if(leftPrev == Grippers.State.CLOSE) Controls.LeftLost = false;
            else Controls.LeftGot = true;
        }
        if(rightPrev != rightGripper.state){
            if(rightPrev == Grippers.State.CLOSE) Controls.RightLost = false;
            else Controls.RightGot = true;
        }


    }

    @Override
    public void runTelemetry(){
        elevator.runTelemetry();
        elevatorArm.runTelemetry();
        leftGripper.runTelemetry("LEFT CLAW");
        rightGripper.runTelemetry("RIGHT CLAW");
        outTakeExtension.runTelemetry();
        ControlHub.telemetry.addData("Outtake state", state.toString());


    }

    public static boolean fullPixel(){
        return rightGripper.state == Grippers.State.CLOSE && leftGripper.state == Grippers.State.CLOSE;
    }

    public static boolean onePixel() {
        return rightGripper.state == Grippers.State.CLOSE || leftGripper.state == Grippers.State.CLOSE;
    }
}
