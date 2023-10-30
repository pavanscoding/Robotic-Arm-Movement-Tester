/*
 * Title: ARM Quick Test
 *
 * Author : Pavan Yeddanapudi
 * 
 * Hardware:
 * The hardware used to test this code includes
 * 3 X Nema 17 Stepper Motor 2A 59Ncm from STEPPERONLINE
 * 3 X TB6600 4A 9-42V Nema 17 Stepper Motor Driver
 * Teensy 4.1 Microcontroller
 * Robotic arm mechanicals
 * Wires, breadboard and power supply
 * PWMServo connected but not tested here.
 *
 * Description:
 * This is the test code to check the movement of the robot with 3 stepper motors.
 * It uses default serial port to take the user input and performs actions on
 * the robotic ARM. This code initializes PWMServo but does not test it.
 */
#include <Arduino.h>
#include "AccelStepper.h"
#include "MultiStepper.h"
#include <PWMServo.h>

/*
 * This is serial port communication baudrate
 */
#define SERIAL_BAUDRATE 115200

/*
 * When using the driver in the hardware to control the stepper motors, we
 * we need to set the interface type as 1 in AccelStepper library.
 */
#define motorInterfaceType 1

/*
 * Teensy 4.1 board pins used for right stepper motor direction and step.
 */
#define dirPin1 4
#define stepPin1 3

/*
 * Teensy 4.1 board pins used for base (center) stepper motor direction and step.
 */
#define dirPin2 7
#define stepPin2 6

/*
 * Teensy 4.1 board pins used for left stepper motor direction and step.
 */
#define dirPin3 11
#define stepPin3 10

/*
 * Teensy 4.1 board pin used for PWMServo.
 */
#define servoPin 1

/*
 * Some Stepper motor intial contants.
 * minPulseWidth is important. As the inernal voltage of Teensy is only 3.3V and
 * drivers generally require around 5V for control signals, making minPulseWidth
 * high value atleast makes sure the control signals are at 3.3V for the drivers.
 */
#define maxSpeed 1000
#define maxAcceleration 500
#define minPulseWidth 15

/*
 * This is set in the hardware driver TB6600.
 * Choose a value based on your application.
 */
#define StepsInRevolution 6400

/*
 * Right Stepper hardware object instatiation in the AccelStepper library
 */
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);

/*
 * Base Stepper hardware object instatiation in the AccelStepper library
 */
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

/*
 * Left Stepper hardware object instatiation in the AccelStepper library
 */
AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);

/*
 * PWMServo initialization.
 */
PWMServo handControl = PWMServo();

/*
 * We use MultiStepper library to move all the 3 stepper motors together.
 */
MultiStepper allsteppers = MultiStepper();


/*
 * Global variable for ARM position. The angle of the steppers are stored in
 * steps. Note we have configured TB6600 steps in one rotation as 6400.
 */
long	arm_base_theta;
long	arm_right_theta;
long	arm_left_theta;


/*
 * This function initalizes the default serial port on Teensy 4.1
 */
void
sd_init(void)
{
  Serial.begin(SERIAL_BAUDRATE);
}


/*
 * This function reads the values of 3 steppers and stores them in the global
 * variables.
 */
void
read_arm_position()
{
  arm_right_theta = stepper1.currentPosition();
  arm_base_theta = stepper2.currentPosition();
  arm_left_theta = stepper3.currentPosition();
}


/*
 * This function moves all 3 steppers to the absolute positions passed in the
 * parameters. Effexctively this function moves the arm to the desired absolute
 * position.
 */
void
move_arm_position(long right, long base, long left)
{
    long pos1[] = {right, base, left};

    allsteppers.moveTo(pos1);

    allsteppers.runSpeedToPosition();
}


/*
 * This function sets all 3 steppers to initial absolute position.
 * When the program starts the 3 steppers position is assume to be
 * (0, 0, 0). All the movements will be relative to this initial
 * posiition. It is called during the initial setup.
 */
void
set_arm_initial_position()
{
  /*
   * Set a sample initial position of that we can see the ARM waking up.
   */
  arm_right_theta = 50;
  arm_base_theta = 50;
  arm_left_theta = 50;

  /*
   * Move the steppers to the initial posiiton.
   */
  move_arm_position(arm_right_theta, arm_base_theta, arm_left_theta);
}


/*
 * This function is a while loop to check the movement of the 3 steppers and
 * ARM. It is called from the mainloop of the code.
 */
void
test_steppers()
{
  char stepper, mvc;
  long mv;

  while(1) { // Forever loop
    read_arm_position(); // Read the current posion of steppers.

    /*
     * Print the posiotn of the ARM read.
     */
    Serial.println("ARM Position (right, base, left):");
    Serial.println(arm_right_theta, DEC);
    Serial.println(arm_base_theta, DEC);
    Serial.println(arm_left_theta, DEC);

    /*
     * Select the stepper you wish to move
     */
    Serial.println("Select Stepper (1 or 2 or 3): ");

    /*
     * Wait for the serial input
     */
    while(Serial.available() == 0)
      ;

    /*
     * Read the serial input
     */
    stepper = Serial.read();

    /*
     * Read and discard other control charecters in the serial port.
     */
    while(Serial.available() != 0)
      Serial.read();
    
    /*
     * Print the stepper number selected.
     */
    Serial.print(stepper);

    /*
     * Fluse the serial port to make sure all the messages are displayed.
     */
    Serial.flush();

    /*
     * Now select the movement of the stepper in the clockwise or counter clockwise
     * direction.
     */
    Serial.println(" Move (1:+100 or 2:-100) = ");

    /*
     * Wait for the serial input
     */
    while(Serial.available() == 0)
      ;
    
    /*
     * Read the serial input
     */
    mvc = Serial.read();

    /*
     * Get the relative steps needed to be moved from the currnt position.
     */
    mv = 0;
    if (mvc == '1') {
      mv = 100;
    }
    if (mvc == '2') {
      mv = -100;
    }

    /*
     * Read and discard other control charecters in the serial port.
     */
    while(Serial.available() != 0)
      Serial.read();
    
    /*
     * Print the mount of steps being moved.
     */
    Serial.println(mv, DEC);

    /*
     * Fluse the serial port to make sure all the messages are displayed.
     */
    Serial.flush();
    
    /*
     * Enable the stepper motors as they were disbaled to save them
     * from heating up when they are not used.
     */
    stepper1.enableOutputs();
    stepper2.enableOutputs();
    stepper3.enableOutputs();
    
    /*
     * Move the arm to the desired posiotion
     */
    if (stepper == '1') {
      move_arm_position(arm_right_theta+mv, arm_base_theta, arm_left_theta);
    }

    if (stepper == '2') {
      move_arm_position(arm_right_theta, arm_base_theta+mv, arm_left_theta);
    }

    if (stepper == '3') {
      move_arm_position(arm_right_theta, arm_base_theta, arm_left_theta+mv);
    }

    delay(600); // Some delay can be decreased.

    /*
     * Disable the stepper motors to save power and stop them from over heating.
     */
    stepper1.disableOutputs();
    stepper2.disableOutputs();
    stepper3.disableOutputs();
  }
}


/*
 * This setup() function is called by the Aurdino platform code
 * during initialization.
 */
void setup() {
  
  /*
   * Initialize the serial port
   */
  sd_init();

  /*
   * Enable the pins on Teensy 4.1 for stepper 1 (right stepper)
   * These two direction and step pins will programed as output.
   */
  stepper1.setEnablePin(dirPin1);
  stepper1.setEnablePin(stepPin1);

  /*
   * Enable the pins on Teensy 4.1 for stepper 2 (base stepper)
   * These two direction and step pins will programed as output.
   */
  stepper2.setEnablePin(dirPin2);
  stepper2.setEnablePin(stepPin2);

  /*
   * Enable the pins on Teensy 4.1 for stepper 3 (left stepper)
   * These two direction and step pins will programed as output.
   */
  stepper3.setEnablePin(dirPin3);
  stepper3.setEnablePin(stepPin3);
    
    /*
     * Initialize the maximum speed and acceration for the 3 steppers.
     * These numbers can changed depending on your application.
     */
    stepper1.setMaxSpeed(maxSpeed);
    stepper1.setAcceleration(maxAcceleration);
    stepper2.setMaxSpeed(maxSpeed);
    stepper2.setAcceleration(maxAcceleration);
    stepper3.setMaxSpeed(maxSpeed);
    stepper3.setAcceleration(maxAcceleration);
    
    /*
     * Add the 3 steppers to the MultiStepper object
     */
    if (!allsteppers.addStepper(stepper1)) {
        Serial.println("ERROR: Failed to add stepper 1 in setup()");
        stepper1.stop();
    }
    if (!allsteppers.addStepper(stepper2)) {
        Serial.println("ERROR: Failed to add stepper 2 in setup()");
        stepper2.stop();
    }
    if (!allsteppers.addStepper(stepper3)) {
        Serial.println("ERROR: Failed to add stepper 3 in setup()");
        stepper3.stop();
    }

    /*
     * Initialize minPulseWidth for all the 3 steppers.
     * This is an important initialization. Make sure minPulseWidth value
     * properly selected for your hardware setup.
     */
    stepper1.setMinPulseWidth(minPulseWidth);
    stepper2.setMinPulseWidth(minPulseWidth);
    stepper3.setMinPulseWidth(minPulseWidth);

    /*
     * Set the ARM in the initial posiotn to wake up the stepper motors.
     */
    set_arm_initial_position();

    /*
     * Disable to the outputs to stop the motors from over heating.
     */
    stepper1.disableOutputs();
    stepper2.disableOutputs();
    stepper3.disableOutputs();

    /*
     * Print something to tell the ARM setup is complete.
     */
    Serial.println("ARM Setup Complete.");

    delay(1000); // Just add some delay can be decreased.

    /*
     * Read the current absolute position of ARM
     */
    read_arm_position();
    
    /*
     * Attach PWMServo. Note it is not tested in this code.
     */
    handControl.attach(servoPin);
}


/*
 * This loop() function is called by the Aurdino platform code
 * during runtime in the mainloop.
 */
void loop() {
  /*
   * Just call the stepper test function.
   */
  test_steppers();
}

