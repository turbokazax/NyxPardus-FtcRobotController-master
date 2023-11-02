package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class zxcqweOpMode extends OpMode {

    /**
     * This file contains an example of an iterative (Non-Linear) "OpMode".
     * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
     * The names of OpModes appear on the menu of the FTC Driver Station.
     * When a selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     * <p>
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all iterative OpModes contain.
     * <p>
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    private DcMotorEx FLmotor = null;
    private DcMotorEx BRmotor = null;
    private DcMotorEx BLmotor = null;
    private DcMotorEx FRmotor = null;



    double kp = 0.008, ki = 0.0, kd = 0.0005;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        /*
        FRmotor -- left back
        BRmotor -- right back
        BLmotor -- left rear
        FLmotor -- right rear
         */

        FRmotor = hardwareMap.get(DcMotorEx.class, "FRmotor");
        FLmotor = hardwareMap.get(DcMotorEx.class, "FLmotor");
        BRmotor = hardwareMap.get(DcMotorEx.class, "BRmotor");
        BLmotor = hardwareMap.get(DcMotorEx.class, "BLmotor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        BLmotor.setDirection(DcMotorEx.Direction.REVERSE);
        FRmotor.setDirection(DcMotor.Direction.FORWARD);
        BRmotor.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        timer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double DriveLeft = -gamepad1.left_stick_y;
        double TurnLeft = -gamepad1.left_stick_x;
        double DriveRight = -gamepad1.right_stick_y;
        double TurnRight = gamepad1.right_stick_x;
//        leftPower = Range.clip(DriveLeft + TurnLeft, -1.0, 1.0);
//        rightPower = Range.clip(DriveRight - TurnRight, -1.0,   1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower  = gamepad1.left_stick_y ;
         rightPower = gamepad1.right_stick_y ;

        // Send calculated power to wheels
        FLmotor.setPower(getPIDPower(FLmotor));
//        FRmotor.setPower(rightPower);
//        BLmotor.setPower(leftPower);
//        BRmotor.setPower(rightPower);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + timer.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double getPIDPower(DcMotorEx motorLeft){
        double accum = 0;
        double prevError = 0;
        double coef = 3000.0;
        double target = 0;
        double prevTime = 0;
        double prevPos = 0;
        double prevTIme = 0;
        double elapsedTime = timer.milliseconds() / 1000.0;
        timer.reset();
        prevTime = elapsedTime;

//            PIDR - потенциальный интегральный дифференцальный регулятор
        target += gamepad1.left_stick_x * 2400 * elapsedTime;
//            coef+=elapsedTime*100;
        double currentPosition = motorLeft.getCurrentPosition();
        prevPos = currentPosition;
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
//        telemetry.addData("power", power);
//        telemetry.addData("target", target);
//        telemetry.addData("current",currentPosition );
//            double prevPos = currentPosition;
//            double prevTime = elapsedTime;

        //          telemetry.addData("ROC", (currentPosition-prevPos)/(elapsedTime - prevTime));
        telemetry.update();
//            double power = powerGotovy;
        return power;
    }
}

/*
        double accum = 0;
        double prevError = 0;
        double coef = 3000.0;
        double target = 0;
        double prevTime = 0;
        waitForStart();
        double prevPos = 0;
        while (opModeIsActive())
        {
//            if(gamepad1.a) motorLeft.setPower(0.0);

            double elapsedTime = timer.milliseconds() / 1000.0;
            timer.reset();
            prevTime = elapsedTime;

//            PIDR - потенциальный интегральный дифференцальный регулятор
            target += gamepad1.left_stick_x * 2400 * elapsedTime;
//            coef+=elapsedTime*100;
            double currentPosition = motorLeft.getCurrentPosition();
            prevPos = currentPosition;
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
//            double prevPos = currentPosition;
//            double prevTime = elapsedTime;

  //          telemetry.addData("ROC", (currentPosition-prevPos)/(elapsedTime - prevTime));
            telemetry.update();
//            double power = powerGotovy;
            motorLeft.setPower(power);

        }
 */

