/* Copyright (c) 2017 FIRST. All rights reserved.
 *
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
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Robot: Teleop Main (Elevator)", group="Robot")
//@Disabled
public class RobotTeleopMain_Eleveator extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  MotorA   = null;
    public DcMotor  MotorB  = null;
    public DcMotor MotorC = null;
    public Servo Servo1 = null;

    //static double speed = 1200;

    //public static PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);
    //public PIDCoefficients pidGains = new PIDCoefficients(0,0,0);

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        double left;
        double right;
        double lift = 0;
        double max;
        double servo = 0;

        int minLift = 0;
        int maxLift = 4260;

        final double K_P = 0.03;


        // Define and Initialize Motors


        MotorA  = hardwareMap.get(DcMotor.class, "MotorA");
        MotorB = hardwareMap.get(DcMotor.class, "MotorB");
        MotorC  = hardwareMap.get(DcMotor.class, "MotorC");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");

        MotorA.setDirection(DcMotor.Direction.REVERSE);
        MotorB.setDirection(DcMotor.Direction.FORWARD);
        MotorC.setDirection(DcMotor.Direction.REVERSE);


        //MotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        MotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int targetPositionLift = 0;



        //targetPositionLift = MotorC.getCurrentPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //PID(speed);

            // Robot Drive Base Speed
            left = gamepad1.left_stick_y*0.4;
            right = gamepad1.right_stick_y*0.5;

            max = Math.abs(left);
            if (max > 1.0)
            {
                left /= max;
            }

            max = Math.abs(right);
            if (max > 1.0)
            {
                right /= max;
            }


            MotorA.setPower(left);
            MotorB.setPower(right);

            if (gamepad1.dpad_down) {
                MotorA.setPower(0.1);
                MotorB.setPower(0.1);
            }
            if (gamepad1.dpad_up) {
                MotorA.setPower(-0.1);
                MotorB.setPower(-0.1);
            }
            if (gamepad1.dpad_right) {
                MotorA.setPower(-0.1);
                MotorB.setPower(0.1);
            }
            if (gamepad1.dpad_left) {
                MotorA.setPower(0.1);
                MotorB.setPower(-0.1);
            }




            if (gamepad1.left_bumper && gamepad1.right_bumper){
                Servo1.setPosition(0.25);
            }
            else if (gamepad1.left_bumper || gamepad1.right_bumper){
                Servo1.setPosition(0.55);
            }






            /*
            if (gamepad1.a) { //open claw
                Servo1.setPosition(0.55);  //decrease to open moreRobotTeleopMain
            }

            if (gamepad1.b) { //close claw
                Servo1.setPosition(0.25);  //increase to close more
            }

             */

            if (gamepad1.y) {
                MotorC.setTargetPosition(2000);
                MotorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MotorC.setPower(1);
            }
            if (gamepad1.x) {
                MotorC.setTargetPosition(2500);
                MotorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                MotorC.setPower(1);
            }


            if (gamepad1.right_trigger != 0 && (MotorC.getCurrentPosition() <= maxLift)){
                MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                MotorC.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger != 0 && (MotorC.getCurrentPosition() >= minLift)) {
                MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                MotorC.setPower(-gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger != 0 && (MotorC.getCurrentPosition() <= maxLift)) {
                MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                MotorC.setPower(0.5);
            }
            else if (gamepad1.left_trigger != 0 && (MotorC.getCurrentPosition() >= minLift)) {
                MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                MotorC.setPower(-0.5);
            }
            else if(MotorC.getCurrentPosition()>maxLift)
            {
                double error = maxLift - MotorC.getCurrentPosition();
                double P = error * K_P;
                MotorC.setPower(Math.tanh(P));
                if (error > 0) {
                    MotorC.setPower(0);
                }

            }
            else if (MotorC.getCurrentPosition() < minLift)
            {
                double error = minLift - MotorC.getCurrentPosition();
                double P = error * K_P;
                MotorC.setPower(Math.tanh(P));
                if (error < 0) {
                    MotorC.setPower(0);
                }

            }
            else{
                MotorC.setPower(0);
            }




            // Send telemetry message to signify robot running;
            telemetry.addData("left: ",  "%.2f", left);
            telemetry.addData("right: ",  "%.2f", right);
            telemetry.addData("A Encoder", "%d", MotorA.getCurrentPosition());
            telemetry.addData("B Encoder", "%d", MotorB.getCurrentPosition());
            telemetry.addData("Elevator Power:",  "%.2f", MotorC.getPower());
            telemetry.addData("lift Current Position: ",  "%d", MotorC.getCurrentPosition());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
    }
