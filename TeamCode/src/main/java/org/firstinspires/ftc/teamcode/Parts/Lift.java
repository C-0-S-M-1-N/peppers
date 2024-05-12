package org.firstinspires.ftc.teamcode.Parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private DcMotor motorLift1, motorLift2, motorLift3;

    private enum lift_positions{
        TOP,
        MIDDLE,
        RESET
    }
    private lift_positions lift;
    public static int speed, trgPosition;

    private Telemetry telemetry;

    public Lift(HardwareMap hardwareMap, Telemetry tel){
        motorLift1 = hardwareMap.get(DcMotor.class, "lift1");
        motorLift2 = hardwareMap.get(DcMotor.class, "lift2");
        motorLift3 = hardwareMap.get(DcMotor.class, "lift3");

        motorLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void init(){
    lift = lift_positions.RESET;
    }
    public void update(){
        
    }
    
}
