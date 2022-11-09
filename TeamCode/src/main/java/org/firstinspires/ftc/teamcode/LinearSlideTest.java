package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="LinearSlideTest", group="Robot")
public class LinearSlideTest extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor leftDrive   = null;
    public DcMotor  rightDrive  = null;

    @Override
    public void runOpMode() {
        double left;

        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorA");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Hello", "Motor test in progress.");

            // Normalize the values so neither exceed +/- 1.0
            left = gamepad1.right_trigger;

            if (left > 1.0) {
                left /= 1.0;
            }

            // Output the safe vales to the motor drives.
            leftDrive.setPower(left);

            telemetry.addData("left",  "%.2f", left);

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}