package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;

public class armAction implements Action {

    DcMotor armMotor;

    int armPos;

    ElapsedTime timer;

    public armAction(DcMotor armMotor, int armPos) {
        this.armMotor = armMotor;
        this.armPos = armPos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        DCMotorUtils.controlMotorWithPID(armMotor, armPos, true);

        return false;
    }
}
