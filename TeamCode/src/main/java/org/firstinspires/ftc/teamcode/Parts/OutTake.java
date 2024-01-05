package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.PixelBed;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoSensor;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class OutTake implements Part {
    public static boolean disable = false;
    public enum STATES{
        EXTEND_TRIGGER,
        EXTEND,
        RETRACT,
        RETRACT_TRIGGER,
        LEVEL_UP,
        LEVEL_DOWN,
        SWAP_PIXELS,
        ROTATE_PIXELS,
        IDLE;
        public static double prevElevatorLevel = 5;
        public static double currentElevatorLevel = 0;
        public static double step = 940/11;
        public static boolean wasSwapped;
    }

    private static double extendArm = 0.68, parallelGround = 0.4;
    private static final double spoolDiameter = 32, armLength = 30, CPR = 145.1;

    public static STATES STATE;
    private Telemetry telemetry;
    private static Controls controls;

    private static Elevator elevator;
    private static ElevatorArm arm;
    private static Grippers leftClaw, rightClaw;
    private static PixelBed pixelBed;

    public OutTake(HardwareMap hm, Controls c, Telemetry tele){
        telemetry = tele;
        controls = c;

        elevator = new Elevator(tele);
        arm = new ElevatorArm(tele);
        pixelBed = new PixelBed(tele);

//        leftClaw = new Grippers(new AutoServo(SERVO_PORTS.S5, true, false, 0, AutoServo.type.MICRO_SERVO),
//                                hm.get(DigitalChannel.class, "eD0"), telemetry);
//        rightClaw = new Grippers(new AutoServo(SERVO_PORTS.S4, true, true, 0, AutoServo.type.MICRO_SERVO),
//                    hm.get(DigitalChannel.class, "eD1"), tele);
        STATE = STATES.IDLE;
        arm.update();
        elevator.update();
        pixelBed.update();
//        leftClaw.update();
//        rightClaw.update();
    }

    private void handleControls(){
        if(STATE != STATES.IDLE) return;
        if(Controls.ExtendElevator) STATE = STATES.EXTEND_TRIGGER;
        if(Controls.RetractElevator) STATE = STATES.RETRACT_TRIGGER;

        if(Controls.ElevatorUp) STATE = STATES.LEVEL_UP;
        if(Controls.ElevatorDown) STATE = STATES.LEVEL_DOWN;
    }

    @Override
    public void update(){
        if(disable) return;
        handleControls();
        double gamma, positionInmm;

        switch (STATE){
            case EXTEND_TRIGGER:
                arm.setPosition(parallelGround);
                STATES.currentElevatorLevel = STATES.prevElevatorLevel;

                gamma = Math.toRadians(arm.getAngle());

                positionInmm = (armLength * Math.sqrt(3)) / (Math.sqrt(3) * Math.cos(gamma) - Math.sin(gamma))
                                    * (1 - Math.sqrt(3) / (Math.sqrt(3) * Math.cos(gamma) + Math.sin(gamma)));

                elevator.setPosition((int) (positionInmm / (spoolDiameter * Math.PI) * CPR));

                if(arm.getPosition() >= parallelGround - 0.01 &&
                    arm.getPosition() <= parallelGround + 0.01){
                    STATE = STATES.EXTEND;
                }
                break;
            case EXTEND:
                arm.setPosition(extendArm);
                elevator.setPosition((int) (STATES.prevElevatorLevel * STATES.step));
                if(elevator.STATE == Elevator.STATES.IDLE){
                    STATE = STATES.IDLE;
                }
                break;
            case IDLE:
                break;
            case LEVEL_UP:
                STATES.currentElevatorLevel++;
                if(STATES.currentElevatorLevel > 11) STATES.currentElevatorLevel = 11;

                elevator.setPosition((int)(STATES.currentElevatorLevel * STATES.step));
                STATE = STATES.IDLE;

                break;
            case LEVEL_DOWN:
                STATES.currentElevatorLevel--;
                if(STATES.currentElevatorLevel < 0) STATES.currentElevatorLevel = 0;

                elevator.setPosition((int)(STATES.currentElevatorLevel * STATES.step));
                STATE = STATES.IDLE;

                break;
            case RETRACT_TRIGGER:
                STATES.prevElevatorLevel = STATES.currentElevatorLevel;
                STATES.wasSwapped = false;
                pixelBed.setHorizontalRotation();

                arm.setPosition(parallelGround);

                if(arm.getPosition() <= 0.01){
                    STATE = STATES.RETRACT;
                    arm.setPosition(0);
                }
                break;
            case RETRACT:
                gamma = Math.toRadians(arm.getAngle());

                positionInmm = (armLength * Math.sqrt(3)) / (Math.sqrt(3) * Math.cos(gamma) - Math.sin(gamma))
                        * (1 - Math.sqrt(3) / (Math.sqrt(3) * Math.cos(gamma) + Math.sin(gamma)));

                elevator.setPosition((int) (positionInmm / (spoolDiameter * Math.PI) * CPR));

                break;
            case SWAP_PIXELS:
                pixelBed.swap();
                break;
            case ROTATE_PIXELS:
                if(STATES.wasSwapped){
                    pixelBed.setHorizontalRotation();
                    STATES.wasSwapped = false;
                } else {
                    pixelBed.setVerticalRotation();
                    STATES.wasSwapped = true;
                }
                break;
        }
        arm.update();
        elevator.update();
        pixelBed.update();
//        leftClaw.update();
//        rightClaw.update();
    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){
        elevator.runTelemetry();
        arm.runTelemetry();
        telemetry.addData("OUT TAKE STATE", STATE.toString());
        telemetry.addData("current level", STATES.currentElevatorLevel);

//        rightClaw.update();
//        leftClaw.update();
    }
}
