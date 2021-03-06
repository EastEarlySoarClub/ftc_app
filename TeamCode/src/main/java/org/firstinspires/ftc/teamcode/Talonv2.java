/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp(name="Talon v2", group="Falcon")
@Disabled
public class Talonv2 extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor arm;
    DcMotor extend;
    DcMotor wrist;

    Servo servoLeft;
    Servo servoRight;
    Servo claw;
    //variables to set the claw open/close (need to be adjusted) "1" means 180 degree rotation
    double leftOpen=0;
    double leftClose=1;
    double rightOpen=1;
    double rightClose=0;
    //variables to open/close the claw
    double clawOpen=0;
    double clawClose=0.5;




    public void start() {

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        arm = hardwareMap.dcMotor.get("arm");
        extend = hardwareMap.dcMotor.get("extend");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");


    }
    @Override
    public void init(){


    }
    @Override
    public void loop(){


        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        float right2 = -gamepad2.right_stick_y;



        right = Range.clip(right, -1, 1);
        left = Range.clip(left,-1,1);
        right2 = Range.clip(right2, -1, 1);

        right = (float)scaleInput(right);
        left = (float)scaleInput(left);
        right2 = (float)scaleInput(right2);


        motorRight.setPower(right);
        motorLeft.setPower(left);
        wrist.setPower(right2);


        // use bumpers to move lift up/down
        if (gamepad1.right_bumper)
        {

            arm.setPower(-.4);

        }

        if (gamepad1.left_bumper)
        {

            arm.setPower(.4);

        }
        if (!gamepad1.left_bumper&&!gamepad1.right_bumper)
        {

            arm.setPower(0);

        }

        // Use gamepad B & X to open/close the big blocks claws
        if (gamepad1.x) {
            servoLeft.setPosition(leftClose);
            servoRight.setPosition(rightClose);
        }
        else if (gamepad1.b) {
            servoLeft.setPosition(leftOpen);
            servoRight.setPosition(rightOpen);
        }

        // Y & A to extend the arm
        if (gamepad1.y)
        {

            extend.setPower(.4);

        }

        if (gamepad1.a)
        {

            extend.setPower(-.4);

        }
        if (!gamepad1.y &&!gamepad1.a)
        {

            extend.setPower(0);

        }
        // Use gamepad dpad up/down to move the wrist
        /*if (gamepad2.right_stick_x)
        {

            wrist.setPower(-.4);

        }

        if (gamepad2.righ)
        {

            wrist.setPower(.4);

        }
        if (!gamepad2.left_bumper&&!gamepad1.right_bumper)
        {

            wrist.setPower(0);

        }
*/



       // dpad left/right to open/close claw
        if (gamepad2.right_bumper)
        {

            claw.setPosition(clawOpen);

        }

        if (gamepad2.left_bumper)
        {

            claw.setPosition(clawClose);

        }






    }



    @Override
    public void stop(){


    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
