package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class zhopaOpMode extends OpMode {

    public static double kp = 0.0, ki = 0.0, kd = 0.0;
    public static double velocity = 0.0;

    private Motor FRmotor = null; // LEFT RIGHT
    private Motor FLmotor = null;
    private Motor BRmotor = null;
    private Motor BLmotor = null;

    private MecanumDrive drive;

    private PIDFController pidFR;
    private PIDFController pidFL;
    private PIDFController pidBR;
    private PIDFController pidBL;


    @Override
    public void init() {
        FRmotor = new Motor(hardwareMap, "FRmotor");
        FLmotor = new Motor(hardwareMap, "FLmotor");
        BRmotor = new Motor(hardwareMap, "BRmotor");
        BLmotor = new Motor(hardwareMap, "BLmotor");
        drive = new MecanumDrive(FLmotor, FRmotor, BLmotor, BRmotor);
//        FRmotor.setRunMode(Motor.RunMode.VelocityControl);
//        FLmotor.setRunMode(Motor.RunMode.VelocityControl);
//        BRmotor.setRunMode(Motor.RunMode.VelocityControl);
//        BLmotor.setRunMode(Motor.RunMode.VelocityControl);
//        //   double kp = 0.008, ki = 0.0, kd = 0.0005;
//        FRmotor.setVeloCoefficients(0.008, 0.0, 0.0005);
//        FLmotor.setVeloCoefficients(0.008, 0.0, 0.0005);
//        BRmotor.setVeloCoefficients(0.008, 0.0, 0.0005);
//        BLmotor.setVeloCoefficients(0.008, 0.0, 0.0005);
        //
        FRmotor.setFeedforwardCoefficients(0.5, 0.1, 1);
        FLmotor.setFeedforwardCoefficients(0.5, 0.1, 1);
        BRmotor.setFeedforwardCoefficients(0.5, 0.1, 1);
        BLmotor.setFeedforwardCoefficients(0.5, 0.1, 1);
        //
        FLmotor.setInverted(true);
        BLmotor.setInverted(true);

    }

    private void updateMotion() {
        double leftPower;
        double rightPower;
        /*double forward = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        double FRpower = forward + lateral + turn;
        double FLpower = forward - lateral - turn;
        double BRpower = forward - lateral + turn;
        double BLpower = forward + lateral - turn;
        drive.driveRobotCentric(lateral, forward, turn);*/

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        double leftFront = power*cos/max + turn;
        double rightFront = power*sin/max - turn;
        double leftRear = power*sin/max + turn;
        double rightRear = power*cos/max - turn;

        if((power+Math.abs(turn))>1){
            leftFront /= power + turn;
            rightFront /= power + turn;
            leftRear /= power + turn;
            rightRear /= power + turn;
        }

        FLmotor.set(leftFront);
        FRmotor.set(rightFront);
        BLmotor.set(leftRear);
        BRmotor.set(rightRear);
        //telemetry:
//        double FRmotorVel = FRmotor.getCorrectedVelocity();
//        double FLmotorVel = FLmotor.getCorrectedVelocity();
//        double BRmotorVel = BRmotor.getCorrectedVelocity();
//        double BLmotorVel = BLmotor.getCorrectedVelocity();
//        telemetry.addData("Vel 0:", FRmotorVel);
//        telemetry.addData("Vel 1:", FLmotorVel);
//        telemetry.addData("Vel 2:", BRmotorVel);
//        telemetry.addData("Vel 3:", BLmotorVel);
//        telemetry.update();

//        double FRtarget = FRpower*velocity;
//        double FLtarget = FLpower*velocity;
//        double BRtarget = BRpower*velocity;
//        double BLtarget = BLpower*velocity;
//        pidFR.setPIDF(kp,ki,kd,0.0);
//        pidFL.setPIDF(kp,ki,kd,0.0);
//        pidBR.setPIDF(kp,ki,kd,0.0);
//        pidBL.setPIDF(kp,ki,kd,0.0);
//        FRmotor.set(pidFR.calculate(FRmotorVel, FRtarget));
//        BRmotor.set(pidFR.calculate(FLmotorVel, FLtarget));
//        FLmotor.set(pidFR.calculate(BRmotorVel, FRtarget));
//        BLmotor.set(pidFR.calculate(BLmotorVel, BLtarget));
    }

    @Override
    public void loop() {
        updateMotion();
    }
}
