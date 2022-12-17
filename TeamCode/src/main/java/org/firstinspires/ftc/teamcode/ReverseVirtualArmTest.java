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
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
Reverse Virtual Arm test
 */
@TeleOp(name= "RVA Test", group="Robot")
public class ReverseVirtualArmTest extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  RVAMotor1   = null;
    public DcMotor  RVAMotor2  = null;
    //public Servo clawFinger = null;

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    //Core Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 4.167;   //gear ratio of gears and sprockets (125t/15t * 20t/40t = 4.167)
    static final double     FIRST_GEAR_DIAMETER_INCH     = 0.3543;    //diameter of starting, 15t gear (CHECK THIS NUMBER!!)
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (FIRST_GEAR_DIAMETER_INCH * Math.PI);

    @Override
    public void runOpMode() {
        double ArmPower = 0;
        String fingerPos = "Closed";

        // Define and Initialize Motors and Servos
        RVAMotor1  = hardwareMap.get(DcMotor.class, "MotorC");
        RVAMotor2 = hardwareMap.get(DcMotor.class, "MotorD");
        //clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

        //core hex motors are facing opposite each other and will rotate in opposite directions
        RVAMotor1.setDirection(DcMotor.Direction.FORWARD);
        RVAMotor2.setDirection(DcMotor.Direction.REVERSE);

        RVAMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RVAMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RVAMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RVAMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RVAMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RVAMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //clawFinger.setPosition(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.left_trigger >= 0.4){
                ArmPower = -0.65;
            }
            else if (gamepad1.right_trigger >= 0.4){
                ArmPower = 0.65;
            }
            else {
                ArmPower = 0;
            }
            RVAMotor1.setPower(ArmPower);
            RVAMotor2.setPower(ArmPower);

            //Handle claw open and close
            //if (gamepad2.left_trigger >= 0.4){
            //    clawFinger.setPosition(0); //close
            //    fingerPos = "Closed";
            //}
            //else if (gamepad2.right_trigger >= 0.4){
            //    clawFinger.setPosition(0.5); //open
             //   fingerPos = "Open";
            //}

            telemetry.addData("Power: ", "%.2f", ArmPower);
            telemetry.addData("RVA Motor A Encoder: %7d", RVAMotor1.getCurrentPosition());
            telemetry.addData("RVA Motor B Encoder: %7d", RVAMotor2.getCurrentPosition());
            telemetry.addData("Distance Traveled (inch) A: %7d", RVAMotor1.getCurrentPosition()/(int)COUNTS_PER_INCH);
            telemetry.addData("Distance Traveled (inch) B: %7d", RVAMotor2.getCurrentPosition()/(int)COUNTS_PER_INCH);
            telemetry.addData("Claw Finger: ", fingerPos);
            telemetry.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}