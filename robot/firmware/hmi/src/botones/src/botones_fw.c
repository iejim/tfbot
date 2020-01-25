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
#define HR_OFF 170000/SPIN_PERIOD
#define HR_2ND 400000/SPIN_PERIOD

#define SYSTEMD_RUNNING "running"
#define SERVICE_CHECK_CYCLES 50 //1/2 second check 

typedef enum status_t {
	STOPPED,
	STARTED,
	RESTARTING
} status_t;

typedef enum led_status_t {
	OFF,
	ON,
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
void systemctl_action(const char* action, const char* service);
void start_service(const char* service);
void restart_service(const char* service);
void stop_service(const char* service);



void start_javabot();
void stop_javabot();
void javabot_is_up();
void javabot_is_down();

void start_teleop();
void stop_teleop();
void teleop_is_up();
void teleop_is_down();

void start_ros();
void stop_ros();
void ros_is_down();
void ros_is_up();



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
led_status_t old_led_green = OFF;
led_status_t led_red = OFF;
led_status_t old_led_red = OFF;

int periods_green = 0;
// int periods_red = 0;

// Used to signal that a button was held, and its release shall be ignored.
int ignore_release = 0;
int service_check_periods = 0;


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
		// fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize PAUSE button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		// fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}

	// initialize MODE button
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		// fprintf(stderr,"ERROR: failed to initialize mode button\n");
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
			// fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_GREEN\n");
			return -1;
	}
	if(rc_led_set(RC_LED_RED, 0)==-1){
			// fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_RED\n");
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

	stop_ros();
	//Exit
	off(GREEN);
	off(RED);


	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	// rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


void spin()
{
	check_main_processes();
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
void on_mode_release()
{
	if(ignore_release)
	{
		ignore_release = 0; //ignored
		return;
	}

	// fprintf(stdout, "PAU Released\n");
	if (status_javabot != STARTED)
		return;
	// fprintf(stdout, "JavaBot was STARTED\n");
	
	stop_javabot();
}
/**
 * Arranca Javabot, si no estaba corriendo, y apaga Teleop, si estaba corriendo.
 */
void on_mode_hold()
{
	// Salir si no se agota el tiempo
	if (!press_wait(MODE))
		return;

	// fprintf(stdout, "PAU Pressed\n");
	ignore_release = 1;

	if (status_javabot == STARTED)
	{	
		// fprintf(stdout, "JavaBot Already STARTED\n");

		return; // No hacer caso

	} else if (status_javabot == STOPPED)
	{
		if (status_teleop == STARTED)
		{
			// fprintf(stdout, "Need to stop Teleop\n");
			stop_teleop();
		}
		// fprintf(stdout, "JavaBot was STOPPED\n");

		start_javabot();
	}
	

}

/*
 * Cambia el status de Teleop entre STARTED y STOPPED.
 */
void on_pause_release()
{
	if(ignore_release)
	{
		ignore_release = 0; //ignored
		return;
	}

	// fprintf(stdout, "PAUSE Released\n");
	
	switch (status_teleop)
	{
	case STARTED:
		// fprintf(stdout, "Teleop is STARTED \n");
		stop_teleop();
		break;

	case STOPPED:
		// fprintf(stdout, "Teleop is STOPPED\n");
		
		start_teleop();
		break;

	case RESTARTING:
		// fprintf(stdout, "Lame Teleop\n");
	default:
		break;
	}
	
}

/*
 * Forces the restart of ROS' TFBOT
 */
void on_pause_hold()
{
	// Salir si no se agota el tiempo
	if (!press_wait(PAUSE))
		return;
	
	// fprintf(stdout, "PAUSE HELD\n");
	ignore_release = 1;

	// Revisar status del servicio
	if (status_ros == STARTED) // Y si simplemente forzamos?
	{
		// fprintf(stdout, "ROS already STARTED: RESTART\n");

		status_ros = RESTARTING;
		stop_ros();
		//Pudiera enviarse para que el main loop lo empiece despues 
		// sino, puede que no tengamos oportunidad de usar LEDs
		// rc_usleep(100000); 
		// start_ros();
		return;
	}
	// fprintf(stdout, "ROS was not STARTED: START\n");
	
	start_ros();


	
}

/**
 * Watchdog process to check for ROS-related services.
 * It would attempt to restart any of them if needed.
 */
void check_main_processes()
{
	
  service_check_periods++;
	if (service_check_periods < SERVICE_CHECK_CYCLES)
		return;

	service_check_periods = 0;
	int status = 0;
	status = check_service(TF);
	if (status == STARTED && status_ros != STARTED ) // Previously down, but running
	{
		ros_is_up();
	}
	else if (status == STOPPED && status_ros == RESTARTING) //Dead, marked for restart
	{
		start_ros();
	} else {
		ros_is_down();
	}
	/*status = 0;
	status = check_service(JavaBot);
	if (status == 1)
	{	
		status_javabot = STARTED;
		javabot_is_up();
	}else
		status_javabot = STOPPED;

	status = 0;
	status = check_service(TeleOp);
	if (status == 1)
		status_teleop = STARTED;
	else
		status_teleop = STOPPED;
		*/

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
	// fprintf(stdout, "Listo para revisar\n");

	if (WIFEXITED(status))
	{
		close(pipefd[1]);
		
		output = fdopen(pipefd[0], "r");
		// fprintf(stdout, "Revisando\n");
		if(fgets(line, sizeof(line), output)) //listen to what tail writes to its standard output
		{
			// fprintf(stdout, "Leyendo salida: %s\n", line);

			char* tok;
			tok = strtok(line,"=");
			if (strcmp(tok, "SubState")==0) //First check
			{
				tok = strtok(NULL,"\n");
				// fprintf(stdout, "Leyendo estado.\n");
				int val = strcmp(tok, SYSTEMD_RUNNING);
				if (val==0) //Final check
				{
					// fprintf(stderr, "Confirmado corriendo: %s is %s\n", service, tok);
					return 1;
				} else { // Not Running
					// fprintf(stderr, "No esta corriendo: %s is %s\n", service , tok);
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

void systemctl_action(const char* action, const char* service){
	pid_t pid = 0;
	int pipefd[2];
	int status;

	pipe(pipefd); //create a pipe
	pid = fork(); //span a child process
	if (pid == 0) // When 0, the child is doing the work.
	{
		// Child. Let's redirect its standard output to our pipe and replace process with tail
		// close(pipefd[0]);
		// dup2(pipefd[1], STDOUT_FILENO);
		// dup2(pipefd[1], STDERR_FILENO);
		execlp("systemctl", "systemctl", action, service, (char*)NULL);
	}

	//Only parent gets here. The rest stays running?
	
	// Wait for the command to exit. It's quick.
	waitpid(pid, &status, 0);

}
void start_service(const char* service)
{
	systemctl_action("start", service);
	
}
void restart_service(const char* service)
{
	systemctl_action("restart", service);

}
void stop_service(const char* service)
{
	systemctl_action("stop", service);

}

void ros_is_up()
{
	led_on(RED);
	// No need to touch green.
	// led_off(GREEN); 
	status_ros = STARTED;
}

void ros_is_down()
{
	led_hr();
	status_ros = STOPPED;
}

void start_ros()
{
	// fprintf(stdout, "Starting Ros\n");

	// Reiniciar servicio
	start_service(TF);
	// fprintf(stdout, "Checking if Ros started\n");
	
	/*if (check_service(TF) == STARTED)
	{
		ros_is_up();

	} else {
		ros_is_down();
	}*/
}

void stop_ros()
{
	// fprintf(stdout, "Stop Ros\n");
	
	/*
	* Fija el LED
	* Detiene el servicio
	*/
	stop_service(TF);
	// Realmente abajo
	/*if (check_service(TF)== STOPPED)
	{
		ros_is_down();
	} else {
		ros_is_up();
	}*/
}


void javabot_is_up()
{
	led_blink4(GREEN);
	status_javabot = STARTED;
}

void javabot_is_down()
{
	status_javabot = STOPPED;
	led_off(GREEN);
}

void start_javabot()
{
	// fprintf(stdout, "Start JavaBot\n");
	
	/*
	* Corre servicio Javabot
	* Pone LED GREEN en blink4
	*/
	/*
	start_service(TF);

	if (check_service(TF) == STARTED)
	{
		javabot_is_up();
	} else {
		javabot_is_down();
	}
	*/

}

void stop_javabot()
{
	// fprintf(stdout, "Stop Javabot\n");
	
	/*
	* Apaga el LED
	* Detiene el servicio
	*/
	/*
	stop_service(JavaBot);
	
	if (check_service(JavaBot)==STOPPED) // Realmente abajo
	{	
		javabot_is_down();
	} else {
		javabot_is_up();
	}
	*/
}

void teleop_is_up()
{
	led_blink2(GREEN);
	status_teleop = STARTED;
}

void teleop_is_down()
{
	status_teleop = STOPPED;
	led_on(GREEN);
}

void start_teleop()
{
	/*
	* Corre servicio Teleop
	* Pone LED GREEN en blink2
	*/
	// fprintf(stdout, "Start Teleop\n");

	start_service(TeleOp);

	if (check_service(TeleOp)==STARTED)
	{
		teleop_is_up();
	} else {
		teleop_is_down();
	}

}

void stop_teleop()
{
	/*
	* Fija el LED
	* Detiene el servicio
	*/
	// fprintf(stdout, "Stop Teleop\n");

	stop_service(TeleOp);
	// Realmente abajo
	if (check_service(TeleOp)==STOPPED)
	{
		teleop_is_down();
	} else {
		teleop_is_up();
	}
	
}


void led_hr()
{	
	// When RED is HR, GREEN must be in no other state
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
			// led_red = OFF; //?
			break;
		
		case RED:
			led_red = BLINK2;
			// led_green = OFF; //?
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
			old_led_green = led_green; 
			led_green = OFF;
			break;
		
		case RED:
			old_led_red = led_red; 
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
	

	switch (led_green)
	{
		case HR:
			// led_red = HR;
			break;
		case ON:
			if (old_led_green != led_green) //First time here
			{
				old_led_green = led_green;
				on(GREEN);
			}
			break;
		case OFF:
			if (old_led_green != led_green) //First time here
			{
				old_led_green = led_green;
				off(GREEN);
			}
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
			if (old_led_red != led_red) //First time here
			{
				old_led_red = led_red;
				on(RED);
			}
			break;
		case OFF:
			if (old_led_red != led_red) //First time here
			{
				old_led_red = led_red;
				off(RED);
			}
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