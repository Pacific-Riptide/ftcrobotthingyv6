package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class HangSpecimenAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;
    Servo claw360;
  //  ElapsedTime timer;

    public HangSpecimenAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota, Servo claw360)
    {
        this.armMotor = armMotor;
        this.rotaMotor = rotaMotor;
        this.clawServo = clawServo;
        this.clawRota = clawRota;
        this.claw360 = claw360;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);

            //DCMotorUtils.controlMotorWithPID(rotaMotor, 100, false);
            Thread.sleep(125);
            DCMotorUtils.controlMotorWithPID(armMotor, -250, true);
            DCMotorUtils.controlMotorWithPID(rotaMotor, 0, false);
            clawRota.setPosition(0.0);
            DCMotorUtils.controlMotorWithPID(armMotor, -100, true);
            Thread.sleep(300);
            //DCMotorUtils.controlMotorWithPID(armMotor, -100, true);
            //Thread.sleep(200);
            //clawRota.setPosition(0.0);
            clawServo.setPosition(RobotConstants.OPEN_CLAW);
        }
        catch (InterruptedException e) {
        }


        return false;
    }
}
