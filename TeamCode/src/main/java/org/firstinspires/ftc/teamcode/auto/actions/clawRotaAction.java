package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class clawRotaAction implements Action {

    Servo clawRota;

    double clawRotaPos;

    ElapsedTime timer;

    public clawRotaAction(Servo clawRota, double clawRotaPos) {
        this.clawRota = clawRota;
        this.clawRotaPos = clawRotaPos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        clawRota.setPosition(clawRotaPos);

        return false;
    }
}