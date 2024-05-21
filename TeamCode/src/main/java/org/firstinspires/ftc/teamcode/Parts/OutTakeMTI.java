package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.OpModes.ArmOnly;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import kotlin.ContextFunctionTypeParams;

@Config
public class OutTakeMTI {
    public enum State{
        WAIT_FOR_PIXELS,
        EXTENDING,
        EXTENDED,
        RETRACTING,
        RETRACTED,
        PLACING_PIXELS,
        PLACE_PURPLE_1,
        PLACE_PURPLE_2,
        RESET_OUTTAKE,
        HANG;
        public static int level = 5;
    }
    public static State state = null;
    public static final int MAX_EXTEND = 1387;
    public static final int STEP = (MAX_EXTEND) / 11;
    public static double safeToExtendOuttake = 40;
    public static double armAnglePlaceingBackboard = 180 - 82, armAnglePlacingPurple = 0,
                        armAngleIntake = 40, armAngleRetracting = 30, Retracted = 46, Extended = 173, intakeRotation = -5;
    public static Elevator elevator = null;
    public static ElevatorArm arm = null;
    public static AutoServo extension = null;
    public static Grippers left = null, right = null;
    private static double pixelsAngle = 0;
    public static double slowmo = 1;

    public OutTakeMTI(){
            extension = new AutoServo(SERVO_PORTS.S3, 0.f, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
            extension.setAngle(Retracted);

            elevator = new Elevator();
            arm = new ElevatorArm();
            state = State.WAIT_FOR_PIXELS;
            left = new Grippers(new AutoServo(SERVO_PORTS.S5, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON),
                    ControlHub.left, 15, 55.f, 125.f);
            right = new Grippers(new AutoServo(SERVO_PORTS.S0, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON),
                    ControlHub.right, 15, 110.f, 35.f);

        align = false;
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
    ElapsedTime armAndExtendTime = new ElapsedTime(), startRetraction = new ElapsedTime();

    private void controls(){
        if(Controls.SetOuttakeToPurplePlacing){
            State.level = 0;
            state = State.PLACE_PURPLE_1;
        }
        if(Controls.Hang) state = State.HANG;
        if(Controls.DropLeft) right.drop();
        if(Controls.DropRight) left.drop();
        if(Controls.rotateRight) {
            arm.rotatePixels(ElevatorArm.Direction.RIGHT);
        }
        if(Controls.rotateLeft){
            arm.rotatePixels(ElevatorArm.Direction.LEFT);
        }

        if(Controls.ExtendElevator) {
            driverUpdated = true;
            if(left.state == Grippers.State.CLOSE) left.close();
            if(right.state == Grippers.State.CLOSE) right.close();
            state = State.EXTENDING;
        }
        if(Controls.RetractElevator && state == State.WAIT_FOR_PIXELS){
            state = State.RESET_OUTTAKE;
        } else {
            state = State.RETRACTING;
            waitForTimer = false;
            startRetraction.reset();
        }
        if(Controls.ElevatorUp){
            State.level++;
            if(State.level > 10) State.level = 10;
            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(State.level * STEP);
        }
        else if(Controls.ElevatorDown){
            State.level--;
            if(State.level < 0) State.level = 0;
            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(State.level * STEP);
        }
    }
    private static boolean driverUpdated = true;
    private ElapsedTime time1 = new ElapsedTime();

    public void update(){
        controls();
        switch (state){
            case WAIT_FOR_PIXELS:
                right.update_values(true);
                left.update_values(true);
                arm.setPixelRotation(intakeRotation);
                arm.setArmAngle(armAngleIntake);
                // sensors responsive
                break;
            case EXTENDING:
                elevator.setTargetPosition(Math.max(safeToExtendOuttake + 10, State.level * STEP));
                arm.setPixelRotation(95);
                arm.setArmAngle(armAnglePlacingPurple); // to be parralel with ground
                state = State.EXTENDED;
                waitForTimer = false;
                time1.reset();
                break;
            case EXTENDED:
                if(elevator.getLivePosition() >= safeToExtendOuttake && !waitForTimer && time1.seconds() >= 0.05 * slowmo){
                    extension.setAngle(Extended);
                    armAndExtendTime.reset();
                    waitForTimer = true;
                }
                if(waitForTimer && armAndExtendTime.seconds() >= 0.5 * slowmo){
                    waitForTimer = false;
                    arm.setPixelRotation(pixelsAngle);
                    elevator.setTargetPosition(STEP * State.level);
                    state = State.PLACING_PIXELS;
                }
                break;
            case RETRACTING:
                driverUpdated = false;
                if(!waitForTimer) {
                    align = false;
                    arm.setArmAngle(armAnglePlacingPurple);
                    arm.setPixelRotation(94);
                    if(startRetraction.seconds() >= 0.4 * slowmo) {
                        extension.setAngle(Retracted);
                        waitForTimer = true;
                        armAndExtendTime.reset();
                    }
                } else if(armAndExtendTime.seconds() >= 0.65 * slowmo){
                    elevator.setTargetPosition(safeToExtendOuttake + 10);
                    state = State.RETRACTED;
                } else if(armAndExtendTime.seconds() >= 0.55 * slowmo){
                    arm.setArmAngle(armAngleRetracting);
                    arm.setPixelRotation(intakeRotation);
                }
                break;
            case RETRACTED:
                if(elevator.reatchedTargetPosition()) {
                    elevator.setTargetPosition(-60);
                    state = State.WAIT_FOR_PIXELS;
                }
                break;
            case PLACING_PIXELS:
                arm.setOrientation(ElevatorArm.rotationAngles[arm.rotationIndex]);
                arm.setArmAngle(armAnglePlaceingBackboard);
                align = true;
                if(left.state == Grippers.State.OPEN && right.state == Grippers.State.OPEN && driverUpdated){
                    if(!dropTime){
                        droping.reset();
                        dropTime = true;
                    } else if(droping.seconds() >= 0.3 * slowmo) {
                        state = State.RETRACTING;
                        dropTime = false;
                    }
                }
                // update dropping
                break;
            case HANG:
                state.level = 5;
                state = State.EXTENDING;
                break;
            case RESET_OUTTAKE:
                elevator.setTargetPosition(safeToExtendOuttake);
                state = State.RETRACTED;
                break;
        }
        if(align)
            arm.setOrientation(ExpansionHub.ImuYawAngle);
        else
            arm.setOrientation(0);

        elevator.update();

        extension.update();

        arm.update();

        left.update();
        right.update();
        elevator.update_values();
        arm.update_values();

        ControlHub.telemetry.addData("state", state.toString());
        right.runTelemetry("right");
        left.runTelemetry("left");
        elevator.runTelemetry();
    }
    private ElapsedTime droping = new ElapsedTime();
    private boolean dropTime = false;
    public static boolean isFullOfPixels(){
        return left.state == Grippers.State.CLOSE && right.state == Grippers.State.CLOSE;
    }
}