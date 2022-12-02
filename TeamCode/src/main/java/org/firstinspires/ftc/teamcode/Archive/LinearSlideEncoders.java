package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//UNTESTED!!

@TeleOp(name="Linear Slide with Encoders", group="Robot")
@Disabled
public class LinearSlideEncoders extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor linearSlide;
    //calculation with NO gear on (hex shaft perimeter 0.684 in, extrusion height ~16.534 in, 3 stages; 80% of max height)
    static final int HIGHEST_JUNCTION_REVOLUTIONS = 58;
    //ticks per revolution
    static final int MOTOR_TICK_COUNT = 288;
    static final int MAX_HEIGHT = MOTOR_TICK_COUNT * HIGHEST_JUNCTION_REVOLUTIONS;

    @Override
    public void runOpMode() {
        int location;

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

            location = linearSlide.getCurrentPosition();

            if (gamepad1.y) { //if y is clicked, move linear slide to max height
                linearSlide.setTargetPosition(MAX_HEIGHT);
                linearSlide.setPower(1.0);
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.a) { //if a is clicked, return to ground
                linearSlide.setTargetPosition(-location);
                linearSlide.setPower(1.0);
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.b) { //if b is clicked, stop linear slide motion
                linearSlide.setPower(0);
            }

            //provide updates
            if  (linearSlide.isBusy()) {
                telemetry.addData("Running", location);
            }
            else {
                telemetry.addData("Stopped", location);
            }
            //telemetry.addData("Revolutions", "%.2f", location/MOTOR_TICK_COUNT);

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}