package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class claw360Action implements Action {

    Servo claw360;

    double claw360Pos;

    ElapsedTime timer;

    public claw360Action(Servo claw360, double claw360Pos) {
        this.claw360 = claw360;
        this.claw360Pos = claw360Pos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        claw360.setPosition(claw360Pos);

        return false;
    }
}