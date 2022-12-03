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
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Turns left 
 */
@Autonomous(name="Auto: Turn Left", group="Robot")
@Disabled
public class AutoScoreOneTurnLeft extends LinearOpMode {
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
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Move forward 19 ticks
        while (rightDrive.getCurrentPosition() < 380) {
            leftDrive.setPower(0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());
            sleep(50);

        }
        //Turn 90 degrees (19 ticks) left
        while (rightDrive.getCurrentPosition() < 980) {
            leftDrive.setPower(-0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());
            sleep(50);
        }
        //Move forward 30 ticks
        while (rightDrive.getCurrentPosition() < 1580) {
            leftDrive.setPower(0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());
            sleep(50);
        }
        //Move backward 19 ticks
        while (rightDrive.getCurrentPosition() > 1200) {
            leftDrive.setPower(-0.3);
            rightDrive.setPower(-0.3);
            telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());
            sleep(50);
        }
        //Turn 90 degrees (19 ticks) right
        while (rightDrive.getCurrentPosition() > 600) {
            leftDrive.setPower(0.3);
            rightDrive.setPower(-0.3);
            telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());
            sleep(50);
        }
        //Move forward 50 ticks
        while (rightDrive.getCurrentPosition() < 1600) {
            leftDrive.setPower(0.3);
            rightDrive.setPower(0.3);
            telemetry.addData("Encoder location:", rightDrive.getCurrentPosition());
            sleep(50);
        }

    }
}