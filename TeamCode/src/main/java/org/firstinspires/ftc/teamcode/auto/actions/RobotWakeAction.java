package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class RobotWakeAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;
    Servo claw360;
    int armHeight;

    public RobotWakeAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota, Servo claw360, int armHeight)
    {
        this.armMotor = armMotor;
        this.rotaMotor = rotaMotor;
        this.clawServo = clawServo;
        this.clawRota = clawRota;
        this.claw360 = claw360;
        this.armHeight = armHeight;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            clawServo.setPosition(0);
            Thread.sleep(50);
            DCMotorUtils.controlMotorWithPID(rotaMotor, 500, false);
            Thread.sleep(50);
            //DCMotorUtils.controlMotorWithPID(armMotor, armHeight, true);
            Thread.sleep(50);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_UP);
        } catch (InterruptedException e) {
        }
        return false;
    }
}

