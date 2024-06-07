package org.firstinspires.ftc.teamcode.Parts;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtensionModule;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.concurrent.atomic.AtomicReference;

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
    public static double armAnglePlaceingBackboard = 165 - 75, armAnglePlacingPurple = 0,
                        armAngleIntake = 30, armAngleRetracting = 15, intakeRotation = 0, tourretOffset = 0;
    public static Elevator elevator = null;
    public static ElevatorArm arm = null;
    public static OutTakeExtensionModule extension = null;
    public static Grippers left = null, right = null;
    private static double pixelsAngle = 0;
    public static double slowmo = 1;
    public static final String cacheFileName = "outTakeStates.pep";
    public static File cacheFile;
    public static void writeStateToFile(State toWrite){
        FileOutputStream out;
        try {
            out = new FileOutputStream(cacheFile);

            out.write((toWrite.toString() + "\n").getBytes(), 0, toWrite.toString().length()+1);

            out.close();
        } catch (IOException e) {
            RobotLog.ee("file exception", "cache file not found, try running \"Create File\" teleOp");
        }
    }
    public static State readStateFromFile(){
        FileInputStream in;
        try {
            in = new FileInputStream(cacheFile);
            InputStreamReader reader = new InputStreamReader(in);
            BufferedReader buffer = new BufferedReader(reader);

            String tmp = buffer.readLine();
            tmp = tmp.replaceAll("[^a-zA-Z0-9_]", "");


            return State.valueOf(tmp);
        } catch (IOException e) {
            RobotLog.ee("file exception", "cache file not found, try running \"Create File\" teleOp");
        }
        return State.WAIT_FOR_PIXELS;

    }

    public OutTakeMTI(){
        cacheFile = new File(Environment.getExternalStorageDirectory(), cacheFileName);
        state = readStateFromFile();
        State.level = 5;
        extension = new OutTakeExtensionModule(SERVO_PORTS.S1, SERVO_PORTS.S2);

        elevator = new Elevator();
        arm = new ElevatorArm();
        state = readStateFromFile();
        left = new Grippers(new AutoServo(SERVO_PORTS.S3, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                    ControlHub.right, 20, 110.f, 180.f);
        right = new Grippers(new AutoServo(SERVO_PORTS.S2, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.MICRO_LEGO),
                    ControlHub.left, 20, 90.f, 10.f);
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
        update();
    }
    public void changeStateTo(State s){
        state = s;
        new Thread(() -> {
            writeStateToFile(s);
        }).start();
    }
    public static boolean align = false, waitForTimer = false;
    ElapsedTime armAndExtendTime = new ElapsedTime(), startRetraction = new ElapsedTime();
    public void setToPurplePlacing(){
        State.level = 0;
        changeStateTo(State.PLACE_PURPLE_1);
    }
    public static boolean hasAPixel(){
        return left.state == Grippers.State.CLOSE || right.state == Grippers.State.CLOSE;
    }
    private void controls(){
        if(Controls.Hang) {
            changeStateTo(State.HANG);
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
            changeStateTo(State.EXTENDING);
        }
        if(Controls.RetractElevator) {
            Controls.RetractElevatorAck = true;
            if (state == State.WAIT_FOR_PIXELS) {
                changeStateTo(State.RESET_OUTTAKE);
                startRetraction.reset();
            } else {
                changeStateTo(State.RETRACTING);
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
    public static boolean driverUpdated = true;
    private ElapsedTime time1 = new ElapsedTime();
    public void setToNormalPlacingFromPurplePixelPlacing(){
        arm.setPixelRotation(pixelsAngle);
        changeStateTo(State.PLACING_PIXELS);
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
                arm.setOrientation(tourretOffset);
                extension.retract();
                // sensors responsive
                break;
            case EXTENDING:
                if(elevator.targetPosition != Math.max(safeToExtendOuttake + 10, State.level * STEP))
                    elevator.setTargetPosition(Math.max(safeToExtendOuttake + 10, State.level * STEP));
                if(time1.seconds() >= 0.2) {
                    arm.setPixelRotation(extendingAngle);
                    arm.setArmAngle(armAnglePlacingPurple); // to be parralel with ground
                    changeStateTo(State.EXTENDED);
                    waitForTimer = false;
                    time1.reset();
                }
                break;
            case EXTENDED:
                if(elevator.getLivePosition() >= safeToExtendOuttake && !waitForTimer && time1.seconds() >= 0.05 * slowmo){
                    extension.extend();
                    armAndExtendTime.reset();
                    waitForTimer = true;
                    time1.reset();
                }
                if(waitForTimer && time1.seconds() >= 0.5 * slowmo){
                    waitForTimer = false;
                    arm.setPixelRotation(pixelsAngle);
                    elevator.setTargetPosition(STEP * State.level);
                    changeStateTo(State.PLACING_PIXELS);
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
                    extension.retract();
                    if(time1.seconds() >= 0.1) {
                        waitForTimer = true;
                        armAndExtendTime.reset();
                    }
                } else if(armAndExtendTime.seconds() >= 0.7 * slowmo){
                    elevator.setTargetPosition(-60);
                    changeStateTo(State.RETRACTED);
                } else if(armAndExtendTime.seconds() >= 0.65 * slowmo){
                    arm.setArmAngle(armAngleRetracting);
                    arm.setPixelRotation(intakeRotation);
                }
                break;
            case RETRACTED:
                if(elevator.reatchedTargetPosition()){
                    changeStateTo(State.WAIT_FOR_PIXELS);
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
                    } else if(droping.seconds() >= 0.3 * slowmo) {
                        dropTime = false;
                        changeStateTo(State.RETRACTING);
                        waitForTimer = false;
                        startRetraction.reset();
                        elevator.setTargetPosition(safeToExtendOuttake);
                    }
                }
                break;
            case HANG:
                State.level = 5;
                changeStateTo(State.EXTENDING);
                break;
            case RESET_OUTTAKE:
                elevator.setTargetPosition(safeToExtendOuttake);
                if(startRetraction.seconds() >= 0.3) {
                    elevator.setTargetPosition(-60);
                }
                if(elevator.reatchedTargetPosition() && elevator.targetPosition == -60){
                    changeStateTo(State.WAIT_FOR_PIXELS);
                }
                break;
            case PLACE_PURPLE_1:
                elevator.setTargetPosition(STEP * 5);
                arm.setPixelRotation(extendingAngle);
                arm.setArmAngle(armAnglePlacingPurple); // to be parralel with ground
                changeStateTo(State.PLACE_PURPLE_2);
                waitForTimer = false;
                time1.reset();
                break;
            case PLACE_PURPLE_2:
                if(elevator.getLivePosition() >= safeToExtendOuttake && extension.isRetracted()){
                    elevator.setTargetPosition(safeToExtendOuttake);
                    extension.extend();
                    time1.reset();
                } else if(extension.isExtended() && time1.seconds() >= 0.55){
                    arm.setPixelRotation(0);
                    elevator.setTargetPosition(-60);
                    extension.extend();
                }
                break;
        }
        if(align)
            arm.setOrientation(ExpansionHub.ImuYawAngle);
        else
            arm.setOrientation(0);

        elevator.update();
        arm.update();

        left.update();
        right.update();
        elevator.update_values();
        arm.update_values();


        ControlHub.telemetry.addData("state", state.toString());
        right.runTelemetry("right");
        left.runTelemetry("left");
        elevator.runTelemetry();
        if(right.state == Grippers.State.CLOSE && rightLast == Grippers.State.OPEN) gotRightPixel = true;
        if(left.state == Grippers.State.CLOSE && leftLast == Grippers.State.OPEN) gotLeftPixel = true;
    }
    private Grippers.State rightLast = Grippers.State.OPEN, leftLast = Grippers.State.OPEN;
    private boolean gotLeftPixel = false, gotRightPixel = false;
    public boolean gotAPixel(){
        if(gotRightPixel){
            gotRightPixel = false;
            return true;
        }
        if(gotLeftPixel){
            gotLeftPixel = false;
            return true;
        }
        return false;
    }
    private ElapsedTime droping = new ElapsedTime();
    private boolean dropTime = false;
    public static boolean isFullOfPixels(){
        return left.state == Grippers.State.CLOSE && right.state == Grippers.State.CLOSE;
    }
}