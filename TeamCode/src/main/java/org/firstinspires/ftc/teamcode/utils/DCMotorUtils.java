package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DCMotorUtils {

    public static void controlMotorWithPID(DcMotor motor, int targetPosition, boolean isArmMotor) {
        int currentPosition = motor.getCurrentPosition();
        int threshold = 10;

        double error, integral = 0, derivative, previousError = 0;
        double kP = 1.0, kI = 0.1, kD = 0.01;

        while (Math.abs(currentPosition - targetPosition) > threshold) {
            error = targetPosition - currentPosition;
            integral += error;
            derivative = error - previousError;
            double output = (kP * error) + (kI * integral) + (kD * derivative);
            previousError = error;

            double adjustedOutput = Math.max(-1.0, Math.min(output, 1.0));
            //Arm motor needs to run in reverse
            //if(isArmMotor){
               // adjustedOutput = -adjustedOutput;
            //}
            motor.setPower(adjustedOutput);

            currentPosition = motor.getCurrentPosition();
        }
        motor.setPower(0);

    }
}
