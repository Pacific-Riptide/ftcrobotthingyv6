package org.firstinspires.ftc.teamcode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class SamplePickupAction implements Action
{
    DcMotor armMotor;
    DcMotor rotaMotor;
    Servo clawServo;
    Servo clawRota;
    Servo claw360;
    int armHeight;

    public SamplePickupAction(DcMotor armMotor, DcMotor rotaMotor, Servo clawServo, Servo clawRota, Servo claw360, int armHeight)
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
           // clawServo.setPosition(RobotConstants.OPEN_CLAW);
           // DCMotorUtils.controlMotorWithPID(armMotor, armHeight);
           // DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_DOWN);
           // Thread.sleep(100);
           // clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
           // clawServo.setPosition(RobotConstants.CLOSED_CLAW);
           // Thread.sleep(100);
           // DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT);
            claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
            clawServo.setPosition(RobotConstants.OPEN_CLAW);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_DOWN - 500, false);
            DCMotorUtils.controlMotorWithPID(armMotor, armHeight, true);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_DOWN, false);
            Thread.sleep(300);
            clawRota.setPosition(RobotConstants.CLAW_ROTA_PERPENDICULAR);
            Thread.sleep(300);
            clawServo.setPosition(RobotConstants.CLOSED_CLAW);
            Thread.sleep(100);
            DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_STRAIGHT, false);

        }
        catch (InterruptedException e)
        {

        }

        return false;
    }
}
