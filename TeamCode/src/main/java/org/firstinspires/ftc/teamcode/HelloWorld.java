package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HelloWorld extends OpMode{
    @Override
    public void init() {
        telemetry.addData("Hello", "World");
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}