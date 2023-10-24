package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class PositionControllingPIDTeleOP extends LinearOpMode {
    // constanti PIDa mojno huyarit' public static
    double kp = 0.001, ki = 0.00, kd = 0.00;

    DcMotorEx motorLeft, motorRight;
    ElapsedTime timer;
//    PIDFController kakoytopid;

    @Override
    public void runOpMode()
    {
//        kakoytopid = new PIDFController(kp, ki, kd, 0);
        timer = new ElapsedTime();

        // init
        motorLeft = hardwareMap.get(DcMotorEx.class, "motor1");

        double accum = 0;
        double prevError = 0;

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.a) motorLeft.setPower(0.0);

            double elapsedTime = timer.milliseconds() / 1000.0;
            timer.reset();

//            PIDR - потенциальный интегральный дифференцальный регулятор
            double target = gamepad1.left_stick_x * motorLeft.getMotorType().getTicksPerRev() * 2;
            double currentPosition = motorLeft.getCurrentPosition();

//            double powerGotovy = kakoytopid.calculate(currentPosition, target);//

            double error = target - currentPosition;

//            P
            double p = error;

//            I
            accum += error;
            double i = accum;

//            D
            double d = (error - prevError) / elapsedTime;
            prevError = error;
//
            double power = p * kp + i * ki + d * kd;
            telemetry.addData("power", power);
            telemetry.addData("target", target);
            telemetry.addData("current",currentPosition );
            telemetry.update();
//            double power = powerGotovy;
            motorLeft.setPower(power);

            motorLeft.getCurrentPosition();
        }
        // loop
    }
}
