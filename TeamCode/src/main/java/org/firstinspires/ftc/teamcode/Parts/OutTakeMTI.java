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
    public static int MAX_LVL = 11;
    public static double safeToExtendOuttake = STEP * 2.5;
    public static double armAnglePlaceingBackboard = 180 - 82, armAnglePlacingPurple = 0,
                        armAngleIntake = 20, armAngleRetracting = 30, Retracted = 10, Extended = 120, intakeRotation = -4;
    public static Elevator elevator = null;
    public static ElevatorArm arm = null;
    public static AutoServo extension = null;
    public static Grippers left = null, right = null;
    private static double pixelsAngle = 0;
    public static double slowmo = 1;

    public OutTakeMTI(){
        State.level = 5;
            extension = new AutoServo(SERVO_PORTS.S1, 0.f, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
            extension.setAngle(Retracted);

            elevator = new Elevator();
            arm = new ElevatorArm();
            state = State.WAIT_FOR_PIXELS;
            left = new Grippers(new AutoServo(SERVO_PORTS.S3, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                    ControlHub.right, 20, 90.f, 155.f);
            right = new Grippers(new AutoServo(SERVO_PORTS.S2, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                    ControlHub.left, 20, 80.f, 10.f);

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
        driverUpdated = false;
    }
    public static boolean align = false, waitForTimer = false;
    ElapsedTime armAndExtendTime = new ElapsedTime(), startRetraction = new ElapsedTime();
    public void setToPurplePlacing(){
        State.level = 0;
        state = State.PLACE_PURPLE_1;
    }
    private void controls(){
        if(Controls.Hang) {
            state = State.HANG;
            Controls.HangAck = true;
        }
        if(Controls.DropLeft) {
            Controls.DropLeftAck = true;
            right.drop();
        }
        if(Controls.DropRight) {
            Controls.DropRightAck = true;
            left.drop();
        }
        if(Controls.rotateRight) {
            Controls.rotateRightAck = true;
            arm.rotatePixels(ElevatorArm.Direction.RIGHT);
        }
        if(Controls.rotateLeft){
            Controls.rotateLeftAck = true;
            arm.rotatePixels(ElevatorArm.Direction.LEFT);
        }

        if(Controls.ExtendElevator) {
            Controls.ExtendElevatorAck = true;
            driverUpdated = true;
            time1.reset();
            if(left.state == Grippers.State.CLOSE) left.close();
            if(right.state == Grippers.State.CLOSE) right.close();
            state = State.EXTENDING;
        }
        if(Controls.RetractElevator) {
            Controls.RetractElevatorAck = true;
            if (state == State.WAIT_FOR_PIXELS) {
                state = State.RESET_OUTTAKE;
                startRetraction.reset();
            } else {
                state = State.RETRACTING;
                waitForTimer = false;
                startRetraction.reset();
            }
        }
        if(Controls.ElevatorUp){
            Controls.ElevatorUpAck = true;
            State.level++;
            if(State.level > MAX_EXTEND) State.level = MAX_EXTEND;
            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(State.level * STEP);
        }
        else if(Controls.ElevatorDown){
            Controls.ElevatorDownAck = true;
            State.level--;
            if(State.level < 0) State.level = 0;
            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(State.level * STEP);
        }
    }
    public void updateElevator(){
        if(State.level > MAX_EXTEND) State.level = MAX_EXTEND;
        elevator.setTargetPosition(State.level * STEP);
    }
    private static boolean driverUpdated = true;
    private ElapsedTime time1 = new ElapsedTime();
    public void setToNormalPlacingFromPurplePixelPlacing(){
        arm.setPixelRotation(pixelsAngle);
        state = State.PLACING_PIXELS;
    }
    private static boolean PixelAckLeft = false, PixelAckRight = false,
            leftPixel = false, rightPixel = false;

    public boolean gotAPixel(){
        if(!PixelAckLeft){
            if(leftPixel) PixelAckLeft = true;
            return leftPixel;
        }
        if(!PixelAckRight){
            if(rightPixel) PixelAckRight = true;
            return PixelAckRight;
        }
        return false;
    }
    public static double extendingAngle = 100;

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
                if(elevator.targetPosition != Math.max(safeToExtendOuttake + 10, State.level * STEP))
                    elevator.setTargetPosition(Math.max(safeToExtendOuttake + 10, State.level * STEP));
                arm.setPixelRotation(extendingAngle);
                if(time1.seconds() >= 0.1) {
                    arm.setArmAngle(armAnglePlacingPurple); // to be parralel with ground
                    state = State.EXTENDED;
                    waitForTimer = false;
                    time1.reset();
                }
                break;
            case EXTENDED:
                if(elevator.getLivePosition() >= safeToExtendOuttake && !waitForTimer && time1.seconds() >= 0.05 * slowmo){
                    extension.setAngle(Extended);
                    armAndExtendTime.reset();
                    waitForTimer = true;
                    time1.reset();
                }
                if(waitForTimer && time1.seconds() >= 0.5 * slowmo){
                    waitForTimer = false;
                    arm.setPixelRotation(pixelsAngle);
                    elevator.setTargetPosition(STEP * State.level);
                    state = State.PLACING_PIXELS;
                    time1.reset();
                }
                break;
            case RETRACTING:
                if(elevator.targetPosition != safeToExtendOuttake) elevator.setTargetPosition(safeToExtendOuttake);
                driverUpdated = false;
                if(!waitForTimer) {
                    align = false;
                    arm.setArmAngle(armAnglePlacingPurple);
                    arm.setPixelRotation(extendingAngle);
                    if(startRetraction.seconds() >= 0.2 * slowmo && elevator.getLivePosition() >= safeToExtendOuttake - 20) {
                        extension.setAngle(Retracted);
                        waitForTimer = true;
                        armAndExtendTime.reset();
                    }
                } else if(armAndExtendTime.seconds() >= 0.85 * slowmo && arm.getArmAngle() == armAngleRetracting){
                    state = State.RETRACTED;
                    elevator.setTargetPosition(-60);
                } else if(armAndExtendTime.seconds() >= 0.7 * slowmo && arm.getArmAngle() != armAngleRetracting){
                    arm.setArmAngle(armAngleRetracting);
                    arm.setPixelRotation(intakeRotation);
                }
                break;
            case RETRACTED:
                if(elevator.reatchedTargetPosition()){
                    state = State.WAIT_FOR_PIXELS;
                }
                break;
            case PLACING_PIXELS:
                arm.setPixelRotation(ElevatorArm.rotationAngles[arm.rotationIndex]);
                arm.setArmAngle(armAnglePlaceingBackboard);
                if(time1.seconds() >= 0.1) {
                    align = true;
                }
                if(left.state == Grippers.State.OPEN && right.state == Grippers.State.OPEN && driverUpdated){
                    if(!dropTime){
                        droping.reset();
                        dropTime = true;
                    } else if(droping.seconds() >= 0.2 * slowmo) {
                        dropTime = false;
                        state = State.RETRACTING;
                        waitForTimer = false;
                        startRetraction.reset();
                        elevator.setTargetPosition(safeToExtendOuttake);
                    }
                }
                break;
            case HANG:
                State.level = 5;
                state = State.EXTENDING;
                break;
            case RESET_OUTTAKE:
                elevator.setTargetPosition(safeToExtendOuttake);
                if(startRetraction.seconds() >= 0.3) {
                    elevator.setTargetPosition(-60);
                }
                if(elevator.reatchedTargetPosition() && elevator.targetPosition == -60){
                    state = State.WAIT_FOR_PIXELS;
                }
                break;
            case PLACE_PURPLE_1:
                elevator.setTargetPosition(STEP * 5);
                arm.setPixelRotation(100);
                arm.setArmAngle(armAnglePlacingPurple); // to be parralel with ground
                state = State.PLACE_PURPLE_2;
                waitForTimer = false;
                time1.reset();
                break;
            case PLACE_PURPLE_2:
                if(elevator.getLivePosition() >= safeToExtendOuttake && extension.getAngle() == Retracted){
                    elevator.setTargetPosition(safeToExtendOuttake);
                    extension.setAngle(Extended);
                    time1.reset();
                } else if(extension.getAngle() == Extended && time1.seconds() >= 0.55){
                    arm.setPixelRotation(0);
                    elevator.setTargetPosition(-69);
                    extension.setAngle(Extended + 0.1);
                }
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