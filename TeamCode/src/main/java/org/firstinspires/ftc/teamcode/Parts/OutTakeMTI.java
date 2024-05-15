package org.firstinspires.ftc.teamcode.Parts;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

public class OutTakeMTI {
    public enum State{
        WAIT_FOR_PIXELS,
        EXTENDING,
        EXTENDED,
        RETRACTING,
        RETRACTED,
        PLACING_PIXELS,
        HANG;
        public int level = 5;
    }
    public static State state = null;
    public static final int MAX_EXTEND = 1341, STEP = MAX_EXTEND / 11;
    public static double safeToExtendOuttake = 100;
    public static double armAnglePlaceingBackboard = 170-96, armAnglePlacingPurple = 0,
                        armAngleIntake = 20, Retracted = 60, Extended = 173, pivotIntake = 185;
    public static Elevator elevator = null;
    public static ElevatorArm arm = null;
    public static AutoServo extension = null;
    public static Grippers left = null, right = null;
    private static double pixelsAngle = 0;

    public OutTakeMTI(){
            extension = new AutoServo(SERVO_PORTS.S3, 0.f, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
            elevator = new Elevator();
            arm = new ElevatorArm();
            state = State.WAIT_FOR_PIXELS;
            left = new Grippers(new AutoServo(SERVO_PORTS.S0, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON),
                    ControlHub.left, 70, 100.f, 25.f);
            right = new Grippers(new AutoServo(SERVO_PORTS.S5, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON),
                    ControlHub.right, 70, 85.f, 10.f);

        align = false;
        extension.setAngle(Retracted);
        arm.setArmAngle(armAngleIntake);
        arm.setPixelRotation(0);
        elevator.setInstantPosition(-60);

        extension.update();
        arm.update();
        elevator.update();
        elevator.update_values();
        left.update();
        right.update();
    }
    public static boolean align = false, waitForTimer = false;
    ElapsedTime armAndExtendTime = new ElapsedTime();

    private void controls(){
        if(Controls.Hang) state = State.HANG;
        if(Controls.DropLeft && state != State.WAIT_FOR_PIXELS) left.drop();
        if(Controls.DropRight && state != State.WAIT_FOR_PIXELS) right.drop();
        if(Controls.rotateRight) {
            pixelsAngle -= 45;
            if(pixelsAngle < -90) pixelsAngle = -90;
            if(state == State.PLACING_PIXELS) arm.setPixelRotation(pixelsAngle);
        }
        if(Controls.rotateLeft){
            pixelsAngle += 45;
            if(pixelsAngle > 90) pixelsAngle = 90;
            if(state == State.PLACING_PIXELS) arm.setPixelRotation(pixelsAngle);
        }

        if(Controls.ExtendElevator)
            state = State.EXTENDING;
        if(Controls.RetractElevator) {
            state = State.RETRACTING;
            waitForTimer = false;
        }
        if(Controls.ElevatorUp){
            state.level ++;
            if(state.level > 10) state.level = 10;
            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(state.level * STEP);
        }
        else if(Controls.ElevatorDown){
            state.level --;
            if(state.level < 0) state.level = 0;
            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(state.level * STEP);
        }
    }

    public void update(){
        controls();
        switch (state){
            case WAIT_FOR_PIXELS:
                left.update();
                right.update();
                // sensors responsive
                break;
            case EXTENDING:
                elevator.setTargetPosition(Math.max(safeToExtendOuttake + 10, state.level * STEP));
                arm.setPixelRotation(94);
                state = State.EXTENDED;
                waitForTimer = false;
                break;
            case EXTENDED:
                if(/*elevator.getLivePosition() >= safeToExtendOuttake &&*/ !waitForTimer){
                    extension.setAngle(Extended);
                    arm.setArmAngle(armAnglePlacingPurple); // to be parralel with ground
                    armAndExtendTime.reset();
                    waitForTimer = true;
                }
                if(waitForTimer && armAndExtendTime.seconds() >= 0.4 /*&& elevator.getLivePosition() >= safeToExtendOuttake*/){
                    align = true;
                    waitForTimer = false;
                    arm.setPixelRotation(pixelsAngle);
                    arm.setArmAngle(armAnglePlaceingBackboard);
                    state = State.PLACING_PIXELS;
                }
                break;
            case RETRACTING:
                if(!waitForTimer) {
                    align = false;
                    arm.setArmAngle(armAngleIntake);
                    arm.setPixelRotation(94);
                    extension.setAngle(Retracted);
                    waitForTimer = true;
                    armAndExtendTime.reset();
                } else if(armAndExtendTime.seconds() >= 0.6){
                    elevator.setTargetPosition(safeToExtendOuttake + 10);
                    arm.setPixelRotation(0);
                    state = State.RETRACTED;
                }
                break;
            case RETRACTED:
                if(/*elevator.reatchedTargetPosition()*/ true) {
                    elevator.setTargetPosition(-60);
                    state = State.WAIT_FOR_PIXELS;
                }
                break;
            case PLACING_PIXELS:
                // update dropping
                break;
            case HANG:
                state.level = 5;
                state = State.EXTENDING;
                break;
        }
        if(align)
            arm.setOrientation(-ExpansionHub.ImuYawAngle);
        else
            arm.setOrientation(0);

//        elevator.update();

        extension.update();

        arm.update();

        elevator.update_values();
        right.update_values();
        left.update_values();
        arm.update_values();

        ControlHub.telemetry.addData("state", state.toString());
        elevator.runTelemetry();
    }
}
