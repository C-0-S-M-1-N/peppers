package org.firstinspires.ftc.teamcode.Parts;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor motorIntake;

    public Intake(HardwareMap hardwareMap, Telemetry tel){
        motorIntake = hardwareMap.get(DcMotor.class, "intake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Work(){
        if(gamepad1.left_bumper)
            motorIntake.setPower(1);
        else if(gamepad1.right_bumper)
            motorIntake.setPower(-1);
    }
}
