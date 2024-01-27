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
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;


@Config
public class OutTake implements Part{
    public static boolean disable = false;
    public static double extendArm = 0.75, bedAngle = 60;
    public enum STATES{
        IDLE(null),
        EXTEND(IDLE),
        EXTEND_TRIGGER(null),
        RETRACT(IDLE),
        RETRACT_TRIGGER(null),
        LVL_UP(IDLE),
        LVL_DOWN(IDLE),
        ROTATE(IDLE),
        SWAP(IDLE);

        public final STATES nextState;
        public final int step = 950/10;
        public static int currentLevel = 4;

        public int MotionSteps = 0;
        public boolean stepDone = false;

        STATES(STATES next){
            nextState = next;
        }
    }
    public STATES STATE;
    public Telemetry telemetry;
    private final Elevator elevator;
    private final ElevatorArm arm;
    private final PixelBed pixelBed;
    public static Grippers LeftClaw, RightClaw;
    private final ElapsedTime timeExtend;
    public static boolean useControls = true;
    public static double offsetTemp = 0.08;

    public OutTake(HardwareMap hm, Telemetry telemetry){
        STATE = STATES.IDLE;
        STATE.MotionSteps = 0;
        STATE.stepDone = false;
        this.telemetry = telemetry;
        elevator = new Elevator(telemetry);
        arm = new ElevatorArm(telemetry);
        pixelBed = new PixelBed(telemetry);
        LeftClaw = new Grippers(new AutoServo(SERVO_PORTS.S4, true, true , 0.01, AutoServo.type.MICRO_SERVO),
                hm.get(DigitalChannel.class, "eD0"), telemetry, "LEFT");
        RightClaw = new Grippers(new AutoServo(SERVO_PORTS.S5, true, false, 0.01, AutoServo.type.MICRO_SERVO),
                hm.get(DigitalChannel.class, "eD1"), telemetry, "RIGHT");
        timeExtend = new ElapsedTime();
        elevator.setPosition(0);
        arm.setAngle(0);
        pixelBed.setBedAngle(0);
        RightClaw.offset = offsetTemp;
        LeftClaw.offsetClose = 0.04;

        elevator.update();
        arm.update();
        pixelBed.update();
        LeftClaw.update();
        RightClaw.update();

    }

    private void controls(){
        if(Controls.DropLeft || Controls.DropRight) {
            LeftClaw.drop();
            RightClaw.drop();
        }

        if(STATE == STATES.IDLE) {
            if (Controls.ElevatorUp) STATE = STATES.LVL_UP;
            if (Controls.ElevatorDown) STATE = STATES.LVL_DOWN;
            if (Controls.ExtendElevator) STATE = STATES.EXTEND_TRIGGER;
            if (Controls.RetractElevator) {STATE = STATES.RETRACT_TRIGGER; timeExtend.reset();}
        }
    }

    @Override
    public void update(){
        if(disable) return;
        controls();
        switch (STATE){
            case IDLE:
                break;
            case LVL_UP:
                if(elevator.getCurrentPosition() <= 2) break;
                STATES.currentLevel++;
                if(STATES.currentLevel > 10) STATES.currentLevel = 10;
                elevator.setPosition(STATES.currentLevel * STATE.step);
                STATE = STATES.IDLE;
                break;
            case LVL_DOWN:
                if(elevator.getCurrentPosition() <= 2) break;
                STATES.currentLevel--;
                if(STATES.currentLevel < 0) STATES.currentLevel = 0;
                STATE = STATES.IDLE;
                elevator.setPosition(STATES.currentLevel * STATE.step);
                break;
            case EXTEND_TRIGGER:
                LeftClaw.manual = true;
                RightClaw.manual = true;
                elevator.setPosition(150);
                pixelBed.setBedAngle(30);

                if(elevator.STATE == Elevator.STATES.IDLE && elevator.getCurrentPosition() != 0){
                    if(timeExtend.seconds() >= 0.25){
                        arm.setAngle(60);
                    }
                    if(timeExtend.seconds() >= 0.4){
                        STATE = STATES.EXTEND;
                    }
                } else timeExtend.reset();
                break;
            case EXTEND:
                elevator.setPosition(STATES.currentLevel * STATE.step);
                arm.setPosition(extendArm);
                pixelBed.setBedAngle(bedAngle);
                if(elevator.STATE == Elevator.STATES.IDLE)
                    STATE = STATES.IDLE;
                break;
            case RETRACT_TRIGGER:
                elevator.setPosition(110);
                arm.setAngle(0);
                pixelBed.setBedAngle(30);
                if(elevator.STATE == Elevator.STATES.IDLE){
                    if(timeExtend.seconds() >= 0.7) {
                        STATE = STATES.RETRACT;
                    }

                } else timeExtend.reset();
                break;
            case RETRACT:
                elevator.setPosition(-2);
                pixelBed.setBedAngle(4);
                Elevator.RETRACTING = true;
                if(elevator.STATE == Elevator.STATES.IDLE && timeExtend.seconds() >= 1){
                    STATE = STATES.IDLE;
                    LeftClaw.manual = false;
                    RightClaw.manual = false;
                    LeftClaw.drop();
                    RightClaw.drop();
                }
                break;

        }
        elevator.update();
        arm.update();
        pixelBed.update();
        LeftClaw.update();
        RightClaw.update();

    }

    @Override
    public void update_values(){
        arm.update_values();
        elevator.update_values();
        pixelBed.update_values();
        LeftClaw.update_values();
        RightClaw.update_values();
    }
    @Override
    public void runTelemetry(){
        arm.runTelemetry();
        elevator.runTelemetry();
        pixelBed.runTelemetry();
        LeftClaw.runTelemetry();
        RightClaw.runTelemetry();

        telemetry.addData("OutTake State", STATE.toString());
        telemetry.addData("motion step", STATE.MotionSteps);
        telemetry.addData("elevator level", STATES.currentLevel);
    }

    public static boolean fullPixel(){
        return RightClaw.STATE == Grippers.STATES.CLOSED &&
               LeftClaw.STATE == Grippers.STATES.CLOSED;
    }
}
