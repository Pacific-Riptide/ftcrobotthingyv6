package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.DCMotorUtils;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class AutonomousRobot extends Robot {
    public void startRobot() throws InterruptedException {
        claw360.setPosition(RobotConstants.CLAW_STRAIGHT);
        DCMotorUtils.controlMotorWithPID(rotaMotor, RobotConstants.ARM_ROTATION_GAME_START_POSITION, false);
        DCMotorUtils.controlMotorWithPID(armMotor, RobotConstants.ARM_DOWN, true);
        clawServo.setPosition(RobotConstants.OPEN_CLAW);
        Thread.sleep(1500);
        clawServo.setPosition(RobotConstants.CLOSED_CLAW);
        Thread.sleep(1000);
        clawRota.setPosition(RobotConstants.CLAW_ROTA_DOWN);
    }

}
