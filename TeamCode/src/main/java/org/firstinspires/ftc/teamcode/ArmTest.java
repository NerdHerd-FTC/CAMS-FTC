package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Arm Test", group="Robot")
public class ArmTest extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor Arm = null;

    @Override
    public void runOpMode() {
        double max;
        double ArmPower;
        final int control = 25;
        //Control (min 1) gives the driver more control over the free speed motor.

        Arm = hardwareMap.get(DcMotor.class, "MotorC");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Arm.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        ArmPower = gamepad1.left_stick_y;
        while (opModeIsActive()) {
            max = Math.abs(Math.pow(ArmPower, 2 * control - 1));
            if (max > 1.0)
            {
                ArmPower /= max;
            }
            
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}