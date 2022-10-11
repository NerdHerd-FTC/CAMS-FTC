package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class HelloWorld extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Send telemetry message to signify robot running;
            telemetry.addLine("Hello World");
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }

        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
