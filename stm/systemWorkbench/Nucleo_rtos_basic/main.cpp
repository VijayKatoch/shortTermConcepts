#include "mbed.h"
#include "mbed_events.h"
#include "pid.h"
#include "STSpin240_250.h"

#define MAX_TUNING_PRM 1

#define MOTOR_TEST 0
#define PID_TEST 1

using namespace KATBOT;

void my_error_handler(uint16_t error);

/* Initialization parameters of the motor connected to the expansion board. */
STSpin240_250_init_t init = {
  20000, /* Frequency of PWM of Input Bridge A in Hz up to 100000Hz     */
  20000, /* Frequency of PWM of Input Bridge B in Hz up to 100000Hz     */
  20000, /* Frequency of PWM used for Ref pin in Hz up to 100000Hz     */
  50, /* Duty cycle of PWM used for Ref pin (from 0 to 100)            */
  TRUE /* Dual Bridge configuration  (FALSE for mono, TRUE for dual brush dc) */
};

typedef struct PID_gain
{
  float kp;
  float ki;
  float kd;
} PID_gain_t;

/* Motor Control Component. */
STSpin240_250 *motor;

static PID_gain_t pDefaultGain[MAX_TUNING_PRM] = {
                                                    { 0.2f, 0.09f, 1.0f }
                                                };

Thread t;
EventQueue queue;

int main()
{

  uint8_t demoStep = 0;

  PID pid = PID(pDefaultGain[0].kp, pDefaultGain[0].ki, pDefaultGain[0].kd);
  pid.setSetPoint(22.5);
  // Start the event queue
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  queue.call_every(pid.getSampleTimeinMS(), &pid, &PID::computeISR);

  /* Printing to the console. */
  printf("STARTING MAIN PROGRAM\r\n");

//----- Initialization 

  /* Initializing Motor Control Component. */
#if (defined TARGET_NUCLEO_F030R8)||(defined TARGET_NUCLEO_F334R8)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A2);
#elif (defined TARGET_NUCLEO_L152RE)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A3);
#elif (defined TARGET_NUCLEO_F429ZI)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D3, A0);
#else
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A0);
#endif
  if (motor->init(&init) != COMPONENT_OK)
    exit(EXIT_FAILURE);

  /* Set dual bridge enabled as two motors are used*/
  motor->set_dual_full_bridge_config(1);

  /* Attaching and enabling an interrupt handler. */
  //motor->attach_flag_irq(&my_flag_irq_handler);
  //motor->enable_flag_irq();
  /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);

  /* Printing to the console. */
  printf("Motor Control Application Example for 2 brush DC motors\r\n");

  /* Set PWM Frequency of Ref to 15000 Hz */
  motor->set_ref_pwm_freq(0, 15000);

  /* Set PWM duty cycle of Ref to 60% */
  motor->set_ref_pwm_dc(0, 60);

  /* Set PWM Frequency of bridge A inputs to 10000 Hz */
  motor->set_bridge_input_pwm_freq(0, 10000);

  /* Set PWM Frequency of bridge B inputs to 10000 Hz */
  motor->set_bridge_input_pwm_freq(1, 10000);

  /* Infinite Loop. */
  printf("--> Infinite Loop...\r\n");
  while (true)
  {
#if MOTOR_TEST
    switch (demoStep)
    {
    case 0:
    {
      printf("STEP 0: Motor(0) FWD Speed=100%% - Motor(1) Inactive\r\n");
      /* Set speed of motor 0 to 100 % */
      motor->set_speed(0, 100);
      /* start motor 0 to run forward*/
      /* if chip is in standby mode */
      /* it is automatically awakened */
      motor->run(0, BDCMotor::FWD);
      break;
    }
    case 1:
    {
      printf("STEP 1: Motor(0) FWD Speed=75%% - Motor(1) BWD Speed=100%%\r\n");
      /* Set speed of motor 0 to 75 % */
      motor->set_speed(0, 75);
      /* Set speed of motor 1 to 100 % */
      motor->set_speed(1, 100);
      /* start motor 1 to run backward */
      motor->run(1, BDCMotor::BWD);
      break;
    }
    case 2:
    {
      printf("STEP 2: Motor(0) FWD Speed=50%% - Motor(1) BWD Speed=75%%\r\n");
      /* Set speed of motor 0 to 50 % */
      motor->set_speed(0, 50);
      /* Set speed of motor 1 to 75% */
      motor->set_speed(1, 75);
      break;
    }
    case 3:
    {
      printf("STEP 3: Motor(0) FWD Speed=25%% - Motor(1) BWD Speed=50%%\r\n");
      /* Set speed of motor 0 to 25 % */
      motor->set_speed(0, 25);
      /* Set speed of motor 1 to 50% */
      motor->set_speed(1, 50);
      break;
    }
    case 4:
    {
      printf("STEP 4: Motor(0) Stopped - Motor(1) BWD Speed=25%%\r\n");
      /* Stop Motor 0 */
      motor->hard_stop(0);
      /* Set speed of motor 1 to 25% */
      motor->set_speed(1, 25);
      break;
    }
    case 5:
    {
      printf("STEP 5: Motor(0) BWD Speed=25%% - Motor(1) Stopped\r\n");
      /* Set speed of motor 0 to 25 % */
      motor->set_speed(0, 25);
      /* start motor 0 to run backward */
      motor->run(0, BDCMotor::BWD);
      /* Stop Motor 1 */
      motor->hard_stop(1);
      break;
    }
    case 6:
    {
      printf("STEP 6: Motor(0) BWD Speed=50%% - Motor(1) FWD Speed=25%%\r\n");
      /* Set speed of motor 0 to 50 % */
      motor->set_speed(0, 50);
      /* Set speed of motor 1 to 25 % */
      motor->set_speed(1, 25);
      /* start motor 1 to run backward */
      motor->run(1, BDCMotor::FWD);
      break;
    }
    case 7:
    {
      printf("STEP 7: Motor(0) BWD Speed=75%% - Motor(1) FWD Speed=50%%\r\n");
      /* Set speed of motor 0 to 75 % */
      motor->set_speed(0, 75);
      /* Set speed of motor 1 to 50 % */
      motor->set_speed(1, 50);
      break;
    }
    case 8:
    {
      printf("STEP 8: Motor(0) BWD Speed=100%% - Motor(1) FWD Speed=75%%\r\n");
      /* Set speed of motor 0 to 100 % */
      motor->set_speed(0, 100);
      /* Set speed of motor 1 to 75 % */
      motor->set_speed(1, 75);
      break;
    }
    case 9:
    {
      printf("STEP 9: Motor(0) BWD Speed=100%% - Motor(1) FWD Speed=100%%\r\n");
      /* Set speed of motor 1 to 100 % */
      motor->set_speed(1, 100);
      break;
    }
    case 10:
    {
      printf("STEP 10\r\n: Stop both motors and disable bridges\r\n");
      /* Stop both motors and disable bridge */
      motor->hard_hiz(0);
      motor->hard_hiz(1);
      break;
    }
    case 11:
    {
      printf(
          "STEP 11: Motor(0) FWD Speed=100%% - Motor(1) FWD Speed=100%%\r\n");
      /* Start both motors to go forward*/
      motor->run(0, BDCMotor::FWD);
      motor->run(1, BDCMotor::FWD);
      break;
    }
    case 12:
    default:
    {
      printf("STEP 12: Stop both motors and enter standby mode\r\n");
      /* Stop both motors and put chip in standby mode */
      motor->reset();
      break;
    }
    }
#endif
#if PID_TEST
    /* Wait for 2 seconds */
    wait_ms(2000);
    printf("pid output = %u\r\n",
        (uint16_t)pid.getOutput());

    pid.setInput(pid.getOutput() * 0.75);
#endif
    /* Increment demostep*/
    demoStep++;
    if (demoStep > 12)
    {
      demoStep = 0;
    }
  }
}

/* Functions -----------------------------------------------------------------*/

/**
 * @brief  This is an example of error handler.
 * @param[in] error Number of the error
 * @retval None
 * @note   If needed, implement it, and then attach it:
 *           + motor->attach_error_handler(&my_error_handler);
 */
void my_error_handler(uint16_t error)
{
  /* Printing to the console. */
  printf("Error %d detected\r\n\n", error);

  /* Infinite loop */
  while (true)
  {
  }
}

/**
 * @brief  This is an example of user handler for the flag interrupt.
 * @param  None
 * @retval None
 * @note   If needed, implement it, and then attach and enable it:
 *           + motor->attach_flag_irq(&my_flag_irq_handler);
 *           + motor->enable_flag_irq();
 *         To disable it:
 *           + motor->DisbleFlagIRQ();
 */
void my_flag_irq_handler(void)
{
  /* Code to be customised */
  /************************/

  printf("    WARNING: \"FLAG\" interrupt triggered.\r\n");

  /* Get the state of bridge A */
  uint16_t bridgeState = motor->get_bridge_status(0);

  if (bridgeState == 0)
  {
    if (motor->get_device_state(0) != INACTIVE)
    {
      /* Bridges were disabled due to overcurrent or over temperature */
      /* When  motor was running */
      my_error_handler(0XBAD0);
    }
  }
}
