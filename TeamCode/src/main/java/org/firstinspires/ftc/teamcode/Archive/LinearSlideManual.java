package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Linear Slide Manual Control", group="Robot")
@Disabled
public class LinearSlideManual extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor linearSlide = null;

    @Override
    public void runOpMode() {
        linearSlide = hardwareMap.get(DcMotor.class, "MotorC");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        linearSlide.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean slideUp = false;
            boolean slideDown = false;

            telemetry.addData("Manual Linear Slide Test", "In Progress");

            //invert stuff
            if (gamepad1.y) {
                slideUp = true;
                slideDown = false;
            }
            else if (gamepad1.a) {
                slideUp = false;
                slideDown = true;
            }
            else if (gamepad1.b) {
                slideDown = false;
                slideUp = false;
            }

            if (slideUp) {
                linearSlide.setPower(1.0);
                telemetry.addData("Direction", "Up");
            }
            else if (slideDown) {
                linearSlide.setPower(-1.0);
                telemetry.addData("Direction", "Down");
            }

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}