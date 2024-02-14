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


        public static final double MAX_EXTEND = 51000;
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
                new AutoServo(SERVO_PORTS.S0, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.MICRO_LEGO),
                hm.get(DigitalChannel.class, "cD0")
        );

        rightGripper = new Grippers(
                new AutoServo(SERVO_PORTS.S1, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.MICRO_LEGO),
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
            if(Controls.ElevatorUp) State.level++;
            if(Controls.ElevatorDown) State.level--;
            if(Controls.DropLeft || Controls.DropRight){
                rightGripper.drop();
                leftGripper.drop();
            }
        }
    }

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
                elevator.setTargetPosition(State.step * 1);
                elevatorArm.setArmAngle(finalArmAngle);
                elevatorArm.setPivotAngle(finalPivotPivotAngle);
                state = State.EXTENDED;
                break;
            case EXTENDED:
                elevator.setTargetPosition(State.level * State.step);
                outTakeExtension.activate();
                align = true;
                state = State.NULL;
                break;
            case RETRACTING:
                align = false;
                outTakeExtension.deactivate();

                elevatorArm.setOrientation(0);
                elevatorArm.setArmAngle(0);
                elevatorArm.update();
                elevatorArm.setPivotAngle(0);
                elevatorArm.update();

                elevator.setTargetPosition(State.step);

                if(elevator.reatchedTargetPosition()){
                    state = State.RETRACTED;
                }

                break;
            case RETRACTED:
                elevator.setTargetPosition(0);
                if(elevator.reatchedTargetPosition())
                    state = State.WAITING_FOR_PIXELS;
                break;
            case NULL:
                break;
        }
        if(align){
            elevatorArm.setOrientation(-ExpansionHub.getRobotRotation(AngleUnit.DEGREES));
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
        ControlHub.telemetry.addData("Outtake state", state.toString());


    }

    public static boolean fullPixel(){
        return rightGripper.state == Grippers.State.CLOSE && leftGripper.state == Grippers.State.CLOSE;
    }
}
