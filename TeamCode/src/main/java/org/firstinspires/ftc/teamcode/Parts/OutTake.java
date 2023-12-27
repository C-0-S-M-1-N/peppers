package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
        public double prevElevatorLevel = 5;
        public double currentElevatorLevel = 0;
        public double step = 950/11;
        public boolean wasSwapped;
    }

    private static double extendArm = 0.75;

    public STATES STATE;
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

        leftClaw = new Grippers(new AutoServo(SERVO_PORTS.S0, false, false, 0, AutoServo.type.MICRO_SERVO),
                                new AutoSensor(hm.get(RevColorSensorV3.class, "sensorLeft"), 20));
        rightClaw = new Grippers(new AutoServo(SERVO_PORTS.S1, false, false, 0, AutoServo.type.GOBILDA),
                new AutoSensor(hm.get(RevColorSensorV3.class, "sensorRight"), 20));

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
        switch (STATE){
            case EXTEND_TRIGGER:
                arm.setPosition(extendArm);
                elevator.setPosition((int) (Math.asin(arm.getPosition())*10));
                if(arm.getPosition() >= extendArm - 0.01 &&
                    arm.getPosition() <= extendArm + 0.01){
                    STATE = STATES.EXTEND;
                }
                break;
            case EXTEND:
                elevator.setPosition((int) (STATE.prevElevatorLevel*STATE.step));
                if(elevator.STATE == Elevator.STATES.IDLE){
                    STATE = STATES.IDLE;
                }
                break;
            case IDLE:
                break;
            case LEVEL_UP:
                STATE.currentElevatorLevel ++;
                if(STATE.currentElevatorLevel > 11) STATE.currentElevatorLevel = 11;

                elevator.setPosition((int)(STATE.currentElevatorLevel * STATE.step));

                break;
            case LEVEL_DOWN:
                STATE.currentElevatorLevel --;
                if(STATE.currentElevatorLevel < 0) STATE.currentElevatorLevel = 0;

                elevator.setPosition((int)(STATE.currentElevatorLevel * STATE.step));

                break;
            case RETRACT_TRIGGER:
                STATE.prevElevatorLevel = STATE.currentElevatorLevel;
                STATE.wasSwapped = false;
                pixelBed.setHorizontalRotation();

                arm.setPosition(0);
                elevator.setPosition((int) (Math.asin(arm.getPosition())*10));

                if(arm.getPosition() <= 0.01){
                    STATE = STATES.RETRACT;
                }
                break;
            case RETRACT:
                elevator.setPosition(0);
                break;
            case SWAP_PIXELS:
                pixelBed.swap();
                break;
            case ROTATE_PIXELS:
                if(STATE.wasSwapped){
                    pixelBed.setHorizontalRotation();
                    STATE.wasSwapped = false;
                } else {
                    pixelBed.setVerticalRotation();
                    STATE.wasSwapped = true;
                }
                break;
        }
        arm.update();
        elevator.update();
        pixelBed.update();
        leftClaw.update();
        rightClaw.update();
    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){
        elevator.runTelemetry();
        arm.runTelemetry();
        rightClaw.update();
        leftClaw.update();
    }
}
