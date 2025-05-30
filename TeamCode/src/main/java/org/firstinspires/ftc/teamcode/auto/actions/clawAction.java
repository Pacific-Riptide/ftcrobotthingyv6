package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class clawAction implements Action {

    Servo clawServo;

    double clawPos;

    ElapsedTime timer;

    public clawAction(Servo clawServo, double clawPos) {
        this.clawServo = clawServo;
        this.clawPos = clawPos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        clawServo.setPosition(clawPos);

        return false;
    }
}