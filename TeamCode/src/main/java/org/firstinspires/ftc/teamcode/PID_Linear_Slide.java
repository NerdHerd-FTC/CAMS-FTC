package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//UNTESTED!!

@TeleOp(name="Linear Slide with Encoders", group="Robot")
public class PID_Linear_Slide extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor linearSlide;
    //calculation with NO gear on (hex shaft perimeter 0.684 in, extrusion height ~16.534 in, 3 stages; 80% of max height)
    static final int HIGHEST_JUNCTION_REVOLUTIONS = 58; //number of revolutions of the motor needed to reach high junction
    static final int MOTOR_TICK_COUNT = 288; //ticks per revolution
    static final int MAX_HEIGHT = MOTOR_TICK_COUNT * HIGHEST_JUNCTION_REVOLUTIONS; //ticks to max height

    @Override
    public void runOpMode() {

        int location;
        int target = 0;
        int error = 0;
        //boolean slideMoving;
        boolean slideDirection; //true = up; false = down
        final double K_P = 2;
        final double K_I = 5;
        final double K_D = 1;
        int integralSum = 0;


        // Define and Initialize Motors
        linearSlide = hardwareMap.get(DcMotor.class, "MotorC");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        linearSlide.setDirection(DcMotor.Direction.REVERSE);

        //sets encoder ticks to zero
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Linear Slide Encoders", "Testing in progress. See updates here.");

            //slideMoving = false;
            if (gamepad1.y) { //if y is clicked, move linear slide to max height
                target = MAX_HEIGHT;
                //slideMoving = true;
                integralSum = 0;
            }
            else if (gamepad1.a) { //if a is clicked, return to ground
                target = 0;
                //slideMoving = true;
                integralSum = 0;

            }

            location = linearSlide.getCurrentPosition();
            if (gamepad1.b) { //if b is pressed, stop linear slide motion
                linearSlide.setPower(0);
            }
            else { //...moreover, ONLY run PID if b is not pressed
                //PID code
                int prevError = error;
                error = target - location;
                //proportional
                double P = K_P * error;
                //integral
                integralSum *= Math.pow(2, -0.050);
                integralSum += error * 50;
                double I = K_I * integralSum;
                //derivative
                double D = K_D * (error-prevError)/50;
                //Set linear slide power using PID
                linearSlide.setPower(P + I + D);
            }

            //provide updates
            if  (linearSlide.isBusy()) {
                telemetry.addData("Running:", "%d%n",location);
            }
            else {
                telemetry.addData("Stopped:", "%d%n", location);
            }

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}