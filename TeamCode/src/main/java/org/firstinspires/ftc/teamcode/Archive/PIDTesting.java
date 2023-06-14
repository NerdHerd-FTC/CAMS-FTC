/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="PID Testing DISABLED", group="Robot")
@Disabled
public class PIDTesting extends LinearOpMode {

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 20;   //gear ratio
    static final double     WHEEL_DIAMETER_INCH     = 3.5;    // For figuring circumference: 90mm
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);
    /* Declare OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        // Define and Initialize Motors and Servos
        leftDrive = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //claw.setPosition(0.0);
        //telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        move(-24);
        move(24);
    }

    private void move(double targetInches) {

        int location = leftDrive.getCurrentPosition();
        int target = (int) (COUNTS_PER_INCH * targetInches) + location;
        double error = 0.05 * (target - location);
        final double limiter = 0.2;
        final long DELTA_T = 20;

        //K constants
        final double K_P = 10;

        while (opModeIsActive()) {
            location = leftDrive.getCurrentPosition();
            error = 0.05 * (target - location) / COUNTS_PER_INCH;

            //Set linear slide power using PID
            double finalPower = K_P * error * limiter;
            leftDrive.setPower(finalPower);
            rightDrive.setPower(finalPower);

            if (Math.abs(error) <= 0.005) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(3 * DELTA_T);
                break;
            }
            telemetry.addData("Error Number", error);
            telemetry.addData("Final Power", finalPower);
            telemetry.addData("Moving", targetInches);
            telemetry.update();
            sleep(DELTA_T);
        }
    }

    private void turn(double targetDegrees) {
        double location = leftDrive.getCurrentPosition();
        double target = (COUNTS_PER_INCH * targetDegrees * 24 / Math.PI) + location;
        double error = 0.05 * (targetDegrees - location);
        final double limiter = 0.2;
        final long DELTA_T = 20;

        //K constants
        final double K_P = 10;

        while (opModeIsActive()) {
            location = leftDrive.getCurrentPosition();
            error = 0.05 * (target - location);

            //Set linear slide power using PID
            double finalPower = K_P * error * limiter;
            leftDrive.setPower(finalPower);
            rightDrive.setPower(-finalPower);

            if (Math.abs(error) <= 0.05) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(3 * DELTA_T);
                break;
            }
            telemetry.addData("Error Number", error);
            telemetry.addData("Final Power", finalPower);
            telemetry.addData("Turning", targetDegrees);
            telemetry.update();
            sleep(DELTA_T);
        }
    }
}