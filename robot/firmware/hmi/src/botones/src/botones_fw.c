/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>
//#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <rc/start_stop.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/time.h>


#define PRESS_TIMEOUT_US 1500000 // button held for 1.5s seconds
#define PRESS_CHECK_US   100000  // check every 1/10 second


typedef enum status_t {
	RUNNING,
	STOPPED,
	RESTARTING
} status_t;

typedef enum led_status_t {
	ON,
	OFF,
	HR,
	BLINK2,
	BLINK4
} led_status_t;

// function declarations

// Pressed for 2 seconds, PAUSE starts JavaBot
// TODO could be that it really "pauses it" (how to communicate with ros?? CLI command?)
void on_pause_press();
// Presed once, PAUSE stops JavaBot
void on_pause_release();
//Pressed for 2 seconds, MODE restarts ROS and useful nodes
void on_mode_press();
// Pressed once, MODE starts / stops TeleOp (an LED should signal it)
void on_mode_release();

int press_wait();


void check_main_processes();
int check_service(const char* service);

void start_service(const char* service);
void restart_service(const char* service);
void stop_service(const char* service);



void start_javabot();
void stop_javabot();

void start_teleop();
void stop_teleop();

void start_ros();
void stop_ros();

// LEDS
// Marcan los estados a usar de los LEDs
void led_hr();
void led_blink2(rc_led_t LED);
void led_blink4(rc_led_t LED);
void led_off(rc_led_t LED);
void led_on(rc_led_t LED);

// Procesa los estados de los leds y actua acorde
void process_leds();
void blink2();
void blink4();
void heart_rate(); // affects both LEDs

// Main Services
const char TF[] = "ros_tfbot";
// const char ST[] = "ros_sabertooth";
// const char Leds[] = "ros_leds";
// const char Servos[] = "ros_servos";

// Runnable
const char JavaBot[] = "ros_javabot";
const char TeleOp[] = "ros_teleop";

// Status
status_t status_javabot = STOPPED;
status_t status_teleop = STOPPED;
status_t status_ros = STOPPED;

#define GREEN RC_LED_GREEN;
#define RED RC_LED_RED;

led_status_t led_green = OFF;
led_status_t led_red = OFF;

/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize PAUSE button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}

	// initialize MODE button
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize mode button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,on_mode_press,on_mode_release);

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start with both LEDs off
	if(rc_led_set(RC_LED_GREEN, 0)==-1){
			fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_GREEN\n");
			return -1;
	}
	if(rc_led_set(RC_LED_RED, 0)==-1){
			fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_RED\n");
			return -1;
	}


	// printf("\nPress and release pause button to turn green LED on and off\n");
	// printf("hold pause button down for 2 seconds to exit\n");

	// Keep looping until state changes to EXITING
	/*
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){

			//Check PID files and stuff

			// roscore must be running
			// - maybe checking any of them would work

			// ROS firmware must be running
				// Sabertooth
				// Servos
				// LEDs
			
		}
		else{
			
		}
		// always sleep at some point
		rc_usleep(100000);
	}*/

	//Exit
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}

int press_wait()
{
	int i=0;
	const int samples = PRESS_TIMEOUT_US/PRESS_CHECK_US;
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(PRESS_CHECK_US);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
				return 0;
		}
	}
	// Time has run out
	return 1;
}

/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if (status_javabot != RUNNING)
		return;

	stop_javabot();
}
/**
 * Arrancar Javabot, si no esta corriendo
 */
void on_pause_press()
{
	// Salir si no se agota el tiempo
	if (!press_wait())
		return;

	if (status_javabot == RUNNING)
	{	
		return; // No hacer caso

	} else if (status_javabot == STOPPED)
	{
		if (status_teleop == RUNNING)
		{
			stop_teleop();
		}

		start_javabot();
	}
	

}

// Toggle Teleop Status
void on_mode_release()
{
	switch (status_teleop)
	{
	case RUNNING:

		stop_teleop();
		break;

	case STOPPED:
		
		start_teleop();
		break;

	case RESTARTING:

	default:
		break;
	}
}

void on_mode_press()
{
	// Salir si no se agota el tiempo
	if (!press_wait())
		return;

	// Revisar status del servicio
	if (status_ros == RUNNING) // Y si simplemente forzamos?
	{
		status_ros = RESTARTING;
		stop_ros();
		rc_usleep(1000); 
		start_ros();
	}


	
}

/**
 * Watchdog process to check for ROS-related services.
 * It would attempt to restart any of them if needed.
 */
void check_main_processes()
{
	// what to do with the return value?
	int status = 0;

	// status = check_service(ST);

}



int check_service(const char* service)
{
	pid_t pid = 0;
	int pipefd[2];
	FILE* output; 
	char line[256];
	int status;

	pipe(pipefd); //create a pipe
	pid = fork(); //span a child process
	if (pid == 0) // When 0, the child is doing the work.
	{
		// Child. Let's redirect its standard output to our pipe and replace process with tail
		close(pipefd[0]);
		dup2(pipefd[1], STDOUT_FILENO);
		// dup2(pipefd[1], STDERR_FILENO);
		execlp("systemctl", "systemctl", "show", "--property=SubState", service, (char*)NULL);
	}

	//Only parent gets here. Listen to what the tail says
	waitpid(pid, &status, 0);
	if (WIFEXITED(status))
	{
		close(pipefd[1]);
		
		output = fdopen(pipefd[0], "r");

		if(fgets(line, sizeof(line), output)) //listen to what tail writes to its standard output
		{
			char* tok;
			tok = strtok(line,"=");
			if (strcmp(tok, "SubState")==0) //First check
			{
				tok = strtok(NULL,"=");
				
				if (strcmp(tok, "running")==0) //Final check
				{
					return 1;
				} else { // Not running
					return 0;
				}
			}
			perror("Error leyendo estado del servicio");
			return -2; // Error reading output
		}

	} else {
		perror("No se pudo leer el estado del servicio.");
	}
	// Forked child EXITED weirdly
	return -1;
}

void start_service(const char* service)
{

}
void restart_service(const char* service)
{

}
void stop_service(const char* service)
{
	
}


void start_ros()
{

	// Reiniciar servicio
	start_service(TF);

	if (check_service(TF) == 1)
	{
		led_on(RED);
		status_ros = RUNNING;

	} else {
		led_hr();
		status_ros = STOPPED;
	}
}

void stop_ros()
{
	/*
	* Fija el LED
	* Detiene el servicio
	*/
	stop_service(TF);
	// Realmente abajo
	if (check_service(TF)==0)
	{
		
		led_hr(GREEN);
		led_hr(RED);
		status_ros = STOPPED;

	} else {

		status_ros = RUNNING;
		led_on(RED);
	}
}

void start_javabot()
{
	/*
	* Corre servicio Javabot
	* Pone LED GREEN en blink4
	*/
	start_service(TF);

	if (check_service(TF))
	{
		led_blink4(GREEN);
		status_javabot = RUNNING;
	} else {
		status_javabot = STOPPED;
		led_off(GREEN);
	}

}

void stop_javabot()
{
	/*
	* Apaga el LED
	* Detiene el servicio
	*/
	stop_service(JavaBot);
	
	if (check_service(JavaBot)==0) // Realmente abajo
	{	
		
		led_off(GREEN);
		status_javabot = STOPPED;

	} else {
		// No se detuvo, simular que corre
		status_javabot = RUNNING;
		led_blink4(GREEN);
	}
	
}

void start_teleop()
{
	/*
	* Corre servicio Teleop
	* Pone LED GREEN en blink2
	*/
	start_service(TeleOp);

	if (check_service(TeleOp)==1)
	{
		led_blink2(GREEN);
		status_teleop = RUNNING;
	} else {
		status_teleop = STOPPED;
		led_off(GREEN);
	}

}

void stop_teleop()
{
	/*
	* Fija el LED
	* Detiene el servicio
	*/
	stop_service(TeleOp);
	// Realmente abajo
	if (check_service(TeleOp)==0)
	{
		
		led_on(GREEN);
		status_teleop = STOPPED;

	} else {

		status_teleop = RUNNING;
		led_blink2(GREEN);
	}
	
}


void led_hr()
{
	led_green = HR;
	led_red = HR;
}

void led_blink2(rc_led_t LED)
{
	// Usar Switch para leer mejor y poder expandir
	switch (LED)
	{
		case GREEN:
			led_green = BLINK2;
			break;
		
		case RED:
			led_red = BLINK2;
			break;

		default:
			// For now, fail silently
			// perror("LED no reconocido");
			break;
	}
}
void led_blink4(rc_led_t LED)
{
	// Usar Switch para leer mejor y poder expandir
	switch (LED)
	{
		case GREEN:
			led_green = BLINK4;
			break;
		
		case RED:
			led_red = BLINK4;
			break;

		default:
			// For now, fail silently
			// perror("LED no reconocido");
			break;
	}
}
void led_off(rc_led_t LED)
{
	switch (LED)
	{
		case GREEN:
			led_green = OFF;
			break;
		
		case RED:
			led_red = OFF;
			break;

		default:
			// For now, fail silently
			// perror("LED no reconocido");
			break;
	}
}

void led_on(rc_led_t LED)
{
	switch (LED)
	{
		case GREEN:
			led_green = ON;
			break;
		
		case RED:
			led_red = ON;
			break;

		default:
			// For now, fail silently
			// perror("LED no reconocido");
			break;
	}
}

/* Processes each LED according to its status. 
 * This runs in the Spinning loop of the program.
 */ 
void process_leds()
{
	// Como se encienden los leds?


}
void blink2()
{

}
void blink4()
{

}
void heart_rate()
{	

}