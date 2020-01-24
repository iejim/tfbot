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
#define SPIN_PERIOD 10000 // 10ms

#define B2_ON 200000/SPIN_PERIOD
#define B2_OFF 300000/SPIN_PERIOD
#define B4_ON 50000/SPIN_PERIOD
#define B4_OFF 250000/SPIN_PERIOD
#define HR_ON 70000/SPIN_PERIOD
#define HR_OFF 250000/SPIN_PERIOD
#define HR_2ND 400000/SPIN_PERIOD

typedef enum status_t {
	STARTED,
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

enum buttons {
	PAUSE,
	MODE
};
// function declarations

// main loop
void spin(); 

/*
 * Pressed for 2 seconds, PAUSE starts JavaBot
 * TODO could be that it really "pauses it" (how to communicate with ros?? CLI command?)
 */
void on_pause_hold();

// Presed once, PAUSE stops JavaBot
void on_pause_release();

//Pressed for 2 seconds, MODE restarts ROS and useful nodes
void on_mode_hold();

// Pressed once, MODE starts / stops TeleOp (an LED should signal it)
void on_mode_release();

int press_wait(int button);


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
void on(rc_led_t LED);
void off(rc_led_t LED);

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

#define GREEN RC_LED_GREEN
#define RED RC_LED_RED

led_status_t led_green = OFF;
led_status_t led_red = OFF;

int periods_green = 0;
// int periods_red = 0;

// Used to signal that a button was held, and its release shall be ignored.
int ignore_release = 0;


/**
 * This template contains these critical components
 * - ensure no existing instances are STARTED and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	// make sure another instance isn't STARTED
	// if return value is -3 then a background process is STARTED with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	// if(rc_kill_existing_process(2.0)<-2) return -1;

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
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_hold,on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,on_mode_hold,on_mode_release);

	// make PID file to indicate your project is STARTED
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	// rc_make_pid_file();

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
	
	rc_set_state(STARTED);
	led_hr(); // Start waiting
	while(rc_get_state()!=EXITING){
		
		
		spin();
	}

	//Exit
	off(GREEN);
	off(RED);

	//Temp
	off(RC_LED_BAT25);
	off(RC_LED_BAT100);

	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	// rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


void spin()
{
	process_leds();

	rc_usleep(SPIN_PERIOD);
}
int press_wait(int button)
{
	int state = 0;
	int i=0;
	const int samples = PRESS_TIMEOUT_US/PRESS_CHECK_US;
	// now keep checking to see if the button is still held down

	for(i=0;i<samples;i++){
		rc_usleep(PRESS_CHECK_US);
		if (button == PAUSE) state = rc_button_get_state(RC_BTN_PIN_PAUSE);
		else if (button == MODE) state = rc_button_get_state(RC_BTN_PIN_MODE);
		
		if(state==RC_BTN_STATE_RELEASED){
				return 0;
		}
	}
	// Time has run out
	return 1;
}

/**
 * Stops Javabot, if it was STARTED.
 */
void on_pause_release()
{
	if(ignore_release)
	{
		ignore_release = 0; //ignored
		return;
	}

	fprintf(stdout, "PAU Released\n");
	if (status_javabot != STARTED)
		return;
	fprintf(stdout, "JavaBot was STARTED\n");
	
	stop_javabot();
}
/**
 * Arranca Javabot, si no estaba corriendo, y apaga Teleop, si estaba corriendo.
 */
void on_pause_hold()
{
	// Salir si no se agota el tiempo
	if (!press_wait(PAUSE))
		return;

	fprintf(stdout, "PAU Pressed\n");
	ignore_release = 1;

	if (status_javabot == STARTED)
	{	
		fprintf(stdout, "JavaBot Already STARTED\n");

		return; // No hacer caso

	} else if (status_javabot == STOPPED)
	{
		if (status_teleop == STARTED)
		{
			fprintf(stdout, "Need to stop Teleop\n");
			stop_teleop();
		}
		fprintf(stdout, "JavaBot was STOPPED\n");

		start_javabot();
	}
	

}

/*
 * Cambia el status de Teleop entre STARTED y STOPPED.
 */
void on_mode_release()
{
	if(ignore_release)
	{
		ignore_release = 0; //ignored
		return;
	}

	fprintf(stdout, "MODE Released\n");
	
	switch (status_teleop)
	{
	case STARTED:
		fprintf(stdout, "Teleop is STARTED \n");
		stop_teleop();
		break;

	case STOPPED:
		fprintf(stdout, "Teleop is STOPPED\n");
		
		start_teleop();
		break;

	case RESTARTING:
		fprintf(stdout, "Lame Teleop\n");
	default:
		break;
	}
	
}

/*
 * Forces the restart of ROS' TFBOT
 */
void on_mode_hold()
{
	// Salir si no se agota el tiempo
	if (!press_wait(MODE))
		return;
	
	fprintf(stdout, "MODE HELD\n");
	ignore_release = 1;

	// Revisar status del servicio
	if (status_ros == STARTED) // Y si simplemente forzamos?
	{
		fprintf(stdout, "ROS already STARTED: RESTART\n");

		status_ros = RESTARTING;
		stop_ros();
		//Pudiera enviarse para que el main loop lo empiece despues 
		// sino, puede que no tengamos oportunidad de usar LEDs
		rc_usleep(1000); 
		start_ros();
		return;
	}
	fprintf(stdout, "ROS was not STARTED: START\n");
	
	start_ros();


	
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
				
				if (strcmp(tok, "STARTED")==0) //Final check
				{
					return 1;
				} else { // Not STARTED
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
	pid_t pid = 0;
	int pipefd[2];

	pipe(pipefd); //create a pipe
	pid = fork(); //span a child process
	if (pid == 0) // When 0, the child is doing the work.
	{
		// Child. Let's redirect its standard output to our pipe and replace process with tail
		// close(pipefd[0]);
		// dup2(pipefd[1], STDOUT_FILENO);
		// dup2(pipefd[1], STDERR_FILENO);
		execlp("systemctl", "systemctl", "start", service, (char*)NULL);
	}

	//Only parent gets here. The rest stays running?
	
}
void restart_service(const char* service)
{
	pid_t pid = 0;
	int pipefd[2];

	pipe(pipefd); //create a pipe
	pid = fork(); //span a child process
	if (pid == 0) // When 0, the child is doing the work.
	{
		// Child. Let's redirect its standard output to our pipe and replace process with tail
		// close(pipefd[0]);
		// dup2(pipefd[1], STDOUT_FILENO);
		// dup2(pipefd[1], STDERR_FILENO);
		execlp("systemctl", "systemctl", "restart", service, (char*)NULL);
	}

	//Only parent gets here. The rest stays running?
}
void stop_service(const char* service)
{
	pid_t pid = 0;
	int pipefd[2];

	pipe(pipefd); //create a pipe
	pid = fork(); //span a child process
	if (pid == 0) // When 0, the child is doing the work.
	{
		// Child. Let's redirect its standard output to our pipe and replace process with tail
		// close(pipefd[0]);
		// dup2(pipefd[1], STDOUT_FILENO);
		// dup2(pipefd[1], STDERR_FILENO);
		execlp("systemctl", "systemctl", "stop", service, (char*)NULL);
	}

	//Only parent gets here. The rest stays running?
}


void start_ros()
{
	fprintf(stdout, "Start Ros\n");

	// Reiniciar servicio
	start_service(TF);
	
	if (check_service(TF) == 1)
	{
		led_on(RED);
		led_off(GREEN);
		status_ros = STARTED;

	} else {
		led_hr();
		status_ros = STOPPED;
	}
}

void stop_ros()
{
	fprintf(stdout, "Stop Ros\n");
	
	/*
	* Fija el LED
	* Detiene el servicio
	*/
	/*stop_service(TF);
	// Realmente abajo
	if (check_service(TF)==0)
	*/if (1) {
		
		led_hr();
		status_ros = STOPPED;
		//Temp
		
		


	} else {

		status_ros = STARTED;
		led_on(RED);
	}
}

void start_javabot()
{
	fprintf(stdout, "Start JavaBot\n");
	
	/*
	* Corre servicio Javabot
	* Pone LED GREEN en blink4
	*/
	/*start_service(TF);

	if (check_service(TF) == 1)
	*/if (1) {
		led_blink4(GREEN);
		status_javabot = STARTED;
	} else {
		status_javabot = STOPPED;
		led_off(GREEN);
	}

}

void stop_javabot()
{
	fprintf(stdout, "Stop Javabot\n");
	
	/*
	* Apaga el LED
	* Detiene el servicio
	*/
	stop_service(JavaBot);
	
	/*if (check_service(JavaBot)==0) // Realmente abajo
	*/if (1) {	
		
		led_off(GREEN);
		status_javabot = STOPPED;
		

	} else {
		// No se detuvo, simular que corre
		status_javabot = STARTED;
		led_blink4(GREEN);
	}
	
}

void start_teleop()
{
	/*
	* Corre servicio Teleop
	* Pone LED GREEN en blink2
	*/
	fprintf(stdout, "Start Teleop\n");

	start_service(TeleOp);

	/*if (check_service(TeleOp)==1)
	*/if (1) {
		led_blink2(GREEN);
		status_teleop = STARTED;
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
	fprintf(stdout, "Stop Teleop\n");

	stop_service(TeleOp);
	// Realmente abajo
	/*if (check_service(TeleOp)==0)
	*/if (1) {
		
		led_on(GREEN);
		status_teleop = STOPPED;
		

	} else {

		status_teleop = STARTED;
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
	

	switch (led_green)
	{
		case HR:
			led_red = HR;
			break;
		case ON:
			on(GREEN);
			break;
		case OFF:
			off(GREEN);
			break;
		case BLINK2:
			blink2();
			// periods_green++;
			break;
		case BLINK4:
			blink4();
			// periods_green++;
			break;
		default: 
			off(GREEN);
		
	}

	switch (led_red)
	{
		case HR:
			// led_green = HR;
			heart_rate();
			// periods_green++;
			break;
		case ON:
			on(RED);
			break;
		case OFF:
			off(RED);
			break;
		case BLINK2:
			blink2();
			break;
		case BLINK4:
			blink4();
			break;
		default: 
			off(RED);
		
	}

}
void blink2()
{
	periods_green++;

	if (periods_green <= B2_ON)
		on(GREEN);
	else if (periods_green < B2_ON + B2_OFF)
		off(GREEN);
	else
		periods_green = 0;
	
}
void blink4()
{
	periods_green++;

	if (periods_green <= B4_ON)
		on(GREEN);
	else if (periods_green <= B4_ON + B4_OFF)
		off(GREEN);
	else
		periods_green = 0;
}
void heart_rate()
{	
	const int T1 = HR_ON + HR_OFF; // OFF
	const int T2 = T1 + HR_ON; // 2nd ON
	const int T3 = T2 + HR_OFF + HR_2ND; // Rest OFF

	periods_green++;
	
	if (periods_green <= HR_ON)
	{	on(GREEN);
		on(RED);
	} else if (periods_green <= T1)
	{	off(GREEN);
		off(RED);
	} else if (periods_green <= T2)
	{	on(GREEN);
		on(RED);
	} else if (periods_green <= T3)
	{	off(GREEN);
		off(RED);
	} else {
		periods_green = 0;
	}
}

void on(rc_led_t LED)
{
	rc_led_set(LED,1);
}

void off(rc_led_t LED)
{
	rc_led_set(LED,0);
}