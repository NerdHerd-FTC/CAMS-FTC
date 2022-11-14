package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LinearSlideTest", group="Robot")
public class LinearSlideTest extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor linearSlide = null;

    @Override
    public void runOpMode() {
        double slideLocation = 0; //tracks the linear slide's location based on how many times the motor has ran

        // Define and Initialize Motors
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
            telemetry.addData("Hello", "Linear Slide test in progress.");

            if (gamepad1.x) {
                if (slideLocation == 0){ //0 represents standstill
                    slideLocation += 1; //get out of standstill to move up
                } else {
                    slideLocation = -slideLocation; //reverse direction if x button is clicked
                }
            }

            if (slideLocation > 0) { //going up
                if (Math.abs(slideLocation) <= 200) { //currently requires a little more than 10 seconds, or 10,000 ms to reach top; based on 50 ms loops
                    linearSlide.setPower(1.0);
                    slideLocation += 1;
                    telemetry.addData("Linear Slide Direction: ", "Up");
                }
            } else { //going down
                if (slideLocation < 0) {
                    linearSlide.setPower(-1.0);
                    slideLocation += 1;
                    telemetry.addData("Linear Slide Direction: ", "Down");
                }
            }

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}