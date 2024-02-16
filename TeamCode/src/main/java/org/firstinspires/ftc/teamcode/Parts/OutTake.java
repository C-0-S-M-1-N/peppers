package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
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
        RETRACTED,
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

    public OutTake(HardwareMap hm){
        align = false;
        state = State.WAITING_FOR_PIXELS;

        elevator = new Elevator();
        elevatorArm = new ElevatorArm();
        outTakeExtension = new OutTakeExtension(
                hm.get(DistanceSensor.class, "sensor"),
                new AutoServo(SERVO_PORTS.S4, 0, true, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON));

        leftGripper = new Grippers(
                new AutoServo(SERVO_PORTS.S2, 0, true, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                hm.get(DigitalChannel.class, "cD0")
        );

        rightGripper = new Grippers(
                new AutoServo(SERVO_PORTS.S3, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                hm.get(DigitalChannel.class, "cD1")
        );


        elevatorArm.setArmAngle(0);
        elevatorArm.setPivotAngle(0);

        elevatorArm.update();
        outTakeExtension.deactivate();
        elevator.setTargetPosition(0);
        state = State.WAITING_FOR_PIXELS;
//        elevator.state = Elevator.State.RESET;
    }

    private void controls(){
        if(Controls.ExtendElevator) state = State.EXTENDING;
        else if(Controls.RetractElevator) state = State.RETRACTING;
        else {
            if(Controls.ElevatorUp) {
                State.level++;
                elevator.setTargetPosition(State.level * State.step);
            }
            if(Controls.ElevatorDown){
                State.level--;
                elevator.setTargetPosition(State.level * State.step);
            }
            if(Controls.DropLeft){
                leftGripper.drop();
            }
            if(Controls.DropRight){
                rightGripper.drop();
            }
        }
    }

    boolean extending = false, set0Pos = false;

    @Override
    public void update(){
        controls();
        if(State.level > 10) State.level = 10;
        if(State.level < 0) State.level = 0;
        switch (state){
            case WAITING_FOR_PIXELS:
                leftGripper.update();
                rightGripper.update();
                break;
            case EXTENDING:
                if(!extending) {
                    elevator.setTargetPosition(State.step * 2);
                    elevatorArm.setArmAngle(10);
                    elevatorArm.setPivotAngle(-5);
                    extending = true;
                } else
                if(elevator.reatchedTargetPosition()) {
                    elevatorArm.setArmAngle(finalArmAngle);
                    elevatorArm.setPivotAngle(finalPivotPivotAngle);
                    state = State.EXTENDED;
                    extending = false;
                }
                break;
            case EXTENDED:
                elevator.setTargetPosition(State.level * State.step);
                align = true;
                state = State.NULL;
                break;
            case RETRACTING:
                if(align) {
                    elevator.setTargetPosition(State.step * 1);
                    outTakeExtension.deactivate();
                    elevatorArm.setOrientation(0);
                    elevatorArm.setArmAngle(0);
                    elevatorArm.setPivotAngle(0);
                }
                align = false;


                if(elevatorArm.reachedStationary()){
                    state = State.RETRACTED;
                }

                break;
            case RETRACTED:
                if(!set0Pos) {
                    elevator.setTargetPosition(-300);
                    set0Pos = true;
                }
                elevator.update();
                if(elevator.reatchedTargetPosition()) {
                    elevator.setTargetPosition(0);
                    state = State.WAITING_FOR_PIXELS;
                    set0Pos = false;
                }
                break;
            case NULL:
                if(elevatorArm.reachedStationary())
                    outTakeExtension.activate();
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
            if(leftPrev == Grippers.State.CLOSE) Controls.LeftLost = true;
            else Controls.LeftGot = true;
        }
        if(rightPrev != rightGripper.state){
            if(rightPrev == Grippers.State.CLOSE) Controls.RightLost = true;
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
}
