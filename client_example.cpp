#include "WProgram.h"
#include <Servo.h>

#include "pins.h"
#include "comms.h"
#include "SHETSource.h"


/******************************************************************************
 * PIN Allocations                                                            *
 ******************************************************************************/

static const int           PIN_SHETSOURCE_READ    = (14 + 4);
static const int           PIN_SHETSOURCE_WRITE   = (14 + 5);

static const int           PIN_SERVO_BOG          = 10;
static const int           PIN_SERVO_ATTIC        = 9;
static const int           PIN_SERVO_DOOR         = 4;

static const int           PIN_DOOR_MAGSWITCH     = 3;
static const int           PIN_DOOR_HANDLE        = 12;

static const int           PIN_PIR_BOG            = 8;
static const int           PIN_PIR_STAIRS         = 7;
static const int           PIN_PIR_ROOM           = 2;

static const int           PIN_LIGHTS_ROOM        = 6;
static const int           PIN_LIGHTS_DESK        = 5;

static const int           PIN_A_TOUCH_Y1         = 0;
static const int           PIN_A_TOUCH_X2         = 1;
static const int           PIN_A_TOUCH_X1         = 2;
static const int           PIN_A_TOUCH_Y2         = 3;


/******************************************************************************
 * Misc Constants                                                             *
 ******************************************************************************/

static const int           SERVO_BOG_ON           = 120;
static const int           SERVO_BOG_OFF          = 90;

static const int           SERVO_ATTIC_ON         = 105;
static const int           SERVO_ATTIC_OFF        = 63;

static const int           SERVO_DOORVO_OPEN      = 10;
static const int           SERVO_DOORVO_CLOSED    = 100;

static const unsigned long PIR_MIN_PERIOD         = 5000;
static const unsigned long TOUCH_RESEND_DELAY     = 100;
static const int           LIGHT_FADE_DURATION    = 1000;
static const unsigned long SERVO_POWERDOWN_TIME   = 1000;

static const unsigned long DOORHANDLE_AVG_DECAY   = 2;
static const unsigned long DOORHANDLE_SENSITIVITY = 2;
static const unsigned long DOORHANDLE_MIN_PERIOD  = 200;
static const unsigned long DOORHANDLE_MAX_ITER    = 100000ul;


/******************************************************************************
 * SHETSource Boiler-Plate                                                    *
 ******************************************************************************/

DirectPins pins = DirectPins(PIN_SHETSOURCE_READ, PIN_SHETSOURCE_WRITE);
Comms comms = Comms(&pins);
SHETSource::Client shetsource = SHETSource::Client(&comms, "jonathan/arduino");


void
shetsource_init()
{
	pins.Init();
	shetsource.Init();
}


void
shetsource_refresh()
{
	shetsource.DoSHET();
}



/******************************************************************************
 * Light Control (Desk/Room)                                                  *
 ******************************************************************************/

typedef struct {
	int           pin;      // Ardunino pin
	int           target;   // Target brightness
	int           old;      // Starting brightness
	unsigned long start;    // Time (millis) fade started
	int           duration; // Total duration of fade
} light_t;

light_t light_room;
light_t light_desk;


void
light_refresh(light_t *light)
{
	if (light->start != 0) {
		unsigned long now = millis();
		
		if (now < light->start + ((unsigned long)light->duration)) {
			// Still fading
			long delta_l = (long)(light->target - light->old);
			long delta_t = (long)(now - light->start);
			
			analogWrite(light->pin,
			            light->old + (int)((delta_t * delta_l)
			                               / ((long)light->duration)));
		} else {
			// Fade finished
			analogWrite(light->pin, light->target);
			light->start = 0;
			light->old = light->target;
		}
	}
}


void
set_lights_room(int value) {
	light_room.old    = light_room.target;
	light_room.target = value;
	light_room.start  = millis();
}


void
set_lights_desk(int value) {
	light_desk.old    = light_desk.target;
	light_desk.target = value;
	light_desk.start  = millis();
}


int get_lights_room(void) { return light_room.target; }
int get_lights_desk(void) { return light_desk.target; }


void
force_lights_room(int value) {
	light_room.old    = value;
	light_room.target = value;
	light_room.start  = 0;
	analogWrite(light_room.pin, value);
}


void
force_lights_desk(int value) {
	light_desk.old    = value;
	light_desk.target = value;
	light_desk.start  = 0;
	analogWrite(light_desk.pin, value);
}


void
lights_init()
{
	pinMode(PIN_LIGHTS_ROOM, OUTPUT);
	pinMode(PIN_LIGHTS_DESK, OUTPUT);
	
	light_room.pin      = PIN_LIGHTS_ROOM;
	light_room.target   = 255;
	light_room.old      = 0;
	light_room.start    = 1;
	light_room.duration = LIGHT_FADE_DURATION;
	
	light_desk.pin      = PIN_LIGHTS_DESK;
	light_desk.target   = 255;
	light_desk.old      = 0;
	light_desk.start    = 1;
	light_desk.duration = LIGHT_FADE_DURATION;
	
	shetsource.AddProperty("light_room", set_lights_room, get_lights_room);
	shetsource.AddProperty("light_desk", set_lights_desk, get_lights_desk);
	
	shetsource.AddAction("light_room_force", force_lights_room);
	shetsource.AddAction("light_desk_force", force_lights_desk);
	
	shetsource.AddProperty("light_room_fade_duration", &(light_room.duration));
	shetsource.AddProperty("light_desk_fade_duration", &(light_desk.duration));
}


void
lights_post_connect()
{
	// Start fading lights on
	light_room.start = millis();
	light_desk.start = millis();
}


void
lights_refresh()
{
	light_refresh(&light_room);
	light_refresh(&light_desk);
}



/******************************************************************************
 * PIR                                                                        *
 ******************************************************************************/

SHETSource::LocalEvent *pir_bog;
SHETSource::LocalEvent *pir_stairs;
SHETSource::LocalEvent *pir_room;

unsigned long last_pir_bog;
unsigned long last_pir_stairs;
unsigned long last_pir_room;


void
pir_init()
{
	pir_bog    = shetsource.AddEvent("pir_bog");
	pir_stairs = shetsource.AddEvent("pir_stairs");
	pir_room   = shetsource.AddEvent("pir_room");
	
	pinMode(PIN_PIR_BOG, INPUT);
	pinMode(PIN_PIR_STAIRS, INPUT);
	pinMode(PIN_PIR_ROOM, INPUT);
	
	// Bog PIR needs pullup, others don't
	digitalWrite(PIN_PIR_BOG, HIGH);
	digitalWrite(PIN_PIR_STAIRS, LOW);
	digitalWrite(PIN_PIR_ROOM, LOW);
}


void
pir_refresh()
{
	unsigned long now = millis();
	
	if (!digitalRead(PIN_PIR_BOG) && (now - last_pir_bog > PIR_MIN_PERIOD)) {
		(*pir_bog)();
		last_pir_bog = now;
	}
	
	if (digitalRead(PIN_PIR_STAIRS) && (now - last_pir_stairs > PIR_MIN_PERIOD)) {
		(*pir_stairs)();
		last_pir_stairs = now;
	}
	
	if (digitalRead(PIN_PIR_ROOM) && (now - last_pir_room > PIR_MIN_PERIOD)) {
		(*pir_room)();
		last_pir_room = now;
	}
}



/******************************************************************************
 * Light Control (Servo-based)                                                *
 ******************************************************************************/

Servo servo_bog;
Servo servo_attic;

int servo_state_bog;
int servo_state_attic;

unsigned int servo_powerdown_bog;
unsigned int servo_powerdown_attic;


void
set_lights_bog(int state)
{
	servo_bog.attach(PIN_SERVO_BOG);
	servo_bog.write(state ? SERVO_BOG_ON : SERVO_BOG_OFF);
	servo_powerdown_bog = millis() + SERVO_POWERDOWN_TIME;
	servo_state_bog = state;
	
	// Hack: Disable stair PIR to combat the drifting grounds...
	last_pir_stairs = millis();
}


void
set_lights_attic(int state)
{
	servo_attic.attach(PIN_SERVO_ATTIC);
	servo_attic.write(state ? SERVO_ATTIC_ON : SERVO_ATTIC_OFF);
	servo_powerdown_attic = millis() + SERVO_POWERDOWN_TIME;
	servo_state_attic = state;
	
	// Hack: Disable stair PIR to combat the drifting grounds...
	last_pir_stairs = millis();
}


int get_lights_bog(void)   { return servo_state_bog; }
int get_lights_attic(void) { return servo_state_attic; }


void
servo_init()
{
	pinMode(PIN_SERVO_BOG, OUTPUT);
	pinMode(PIN_SERVO_ATTIC, OUTPUT);
	
	digitalWrite(PIN_SERVO_BOG, LOW);
	digitalWrite(PIN_SERVO_ATTIC, LOW);
	
	shetsource.AddProperty("light_bog",   set_lights_bog,   get_lights_bog);
	shetsource.AddProperty("light_attic", set_lights_attic, get_lights_attic);
	
	servo_powerdown_bog = 0;
	servo_powerdown_attic = 0;
}


void
servo_refresh()
{
	unsigned int now = millis();
	
	if (servo_powerdown_bog != 0 && servo_powerdown_bog < now) {
		servo_bog.detach();
		digitalWrite(PIN_SERVO_BOG, LOW);
	}
	
	if (servo_powerdown_attic != 0 && servo_powerdown_attic < now) {
		servo_attic.detach();
		digitalWrite(PIN_SERVO_ATTIC, LOW);
	}
}



/******************************************************************************
 * Touchpanel                                                                 *
 ******************************************************************************/

SHETSource::LocalEvent *touch_move;
SHETSource::LocalEvent *touch_up;
SHETSource::LocalEvent *touch_down;

// XXX: Awful "legacy code which works so lets not fix it" follows

bool touch_x_mode;

void
set_touch_x_mode(bool x_mode)
{
	touch_x_mode = x_mode;
	if (x_mode == true) {
		     pinMode(PIN_A_TOUCH_X1+14, OUTPUT);      pinMode(PIN_A_TOUCH_X2+14, OUTPUT);
		digitalWrite(PIN_A_TOUCH_X1+14, HIGH);   digitalWrite(PIN_A_TOUCH_X2+14, LOW);
		
		     pinMode(PIN_A_TOUCH_Y1+14, INPUT);       pinMode(PIN_A_TOUCH_Y2+14, INPUT);
		digitalWrite(PIN_A_TOUCH_Y1+14, HIGH);   digitalWrite(PIN_A_TOUCH_Y2+14, HIGH);
	} else {
		     pinMode(PIN_A_TOUCH_Y1+14, OUTPUT);      pinMode(PIN_A_TOUCH_Y2+14, OUTPUT);
		digitalWrite(PIN_A_TOUCH_Y1+14, LOW);    digitalWrite(PIN_A_TOUCH_Y2+14, HIGH);
		
		     pinMode(PIN_A_TOUCH_X1+14, INPUT);       pinMode(PIN_A_TOUCH_X2+14, INPUT);
		digitalWrite(PIN_A_TOUCH_X1+14, HIGH);   digitalWrite(PIN_A_TOUCH_X2+14, HIGH);
	}
}

int
touch_read()
{
	int sig = analogRead((touch_x_mode ? PIN_A_TOUCH_Y1 : PIN_A_TOUCH_X1));
	return sig >= 970 ? -1 : sig;
}


void
touch_init()
{
	touch_move = shetsource.AddEvent("touch_move");
	touch_up   = shetsource.AddEvent("touch_up");
	touch_down = shetsource.AddEvent("touch_down");
	
	set_touch_x_mode(true);
}


void
touch_refresh()
{
	static unsigned long int last_press = 0;
	static uint16_t last_pos = 0;
	static int down = 0;
	
	long int y = touch_read();
	if (y == -1) {
		if (down == 1) {
			(*touch_up)(last_pos);
			down = 0;
		}
		return;
	}
	
	// Allow stabalisation
	delay(10);
	y = touch_read();
	if (y == -1) return;
	y = 255-(((y-100)*255)/900);
	
	set_touch_x_mode(false);
	delay(10);
	long int x = touch_read();
	set_touch_x_mode(true);
	if (x == -1) return;
	x = ((x-150)*255)/800;
	
	if (x < 0 || y < 0)
		return;
	
	uint16_t val;
	val = y&0xFF;
	val |= (x&0xFF)<<8;
	last_pos = val;
	
	if (down == 0) {
		down = 1;
		(*touch_down)(val);
	}
	
	if (last_press + TOUCH_RESEND_DELAY < millis()) {
		last_press = millis();
		(*touch_move)(val);
	}
}



/******************************************************************************
 * Door & Door-Handle                                                         *
 ******************************************************************************/

SHETSource::LocalEvent *door_closed;
SHETSource::LocalEvent *door_opened;
SHETSource::LocalEvent *door_handle_touched;


unsigned long doorhandle_avg;
unsigned long last_door_handle_touch;


int
is_door_open(void)
{
	return digitalRead(PIN_DOOR_MAGSWITCH) == HIGH;
}


unsigned long
doorhandle_sense(void)
{
	// Pull the wire to ground
	pinMode(PIN_DOOR_HANDLE, OUTPUT);
	digitalWrite(PIN_DOOR_HANDLE, LOW);
	delay(2);
	
	// Disconnect and time until pulled back up
	pinMode(PIN_DOOR_HANDLE, INPUT);
	unsigned long time = micros();
	unsigned long counter = 0;
	while ((digitalRead(PIN_DOOR_HANDLE) != HIGH) && (counter++ < DOORHANDLE_MAX_ITER))
		;
	
	return micros() - time;
}


void
door_init()
{
	door_closed         = shetsource.AddEvent("door_closed");
	door_opened         = shetsource.AddEvent("door_opened");
	door_handle_touched = shetsource.AddEvent("door_handle_touched");
	
	shetsource.AddAction("is_door_open", is_door_open);
	
	// Mag switch is pulled low when the door closed
	pinMode(PIN_DOOR_MAGSWITCH, INPUT);
	digitalWrite(PIN_DOOR_MAGSWITCH, HIGH);
	
	// Handle sensing -- leave floating
	pinMode(PIN_DOOR_HANDLE, INPUT);
	digitalWrite(PIN_DOOR_HANDLE, LOW);
	
	// Initialise the average doorhandle value
	doorhandle_avg = doorhandle_sense();
	
	last_door_handle_touch = 0;
}


void
door_refresh()
{
	// Monitor the door
	static bool door_open = false;
	bool new_door_open = is_door_open();
	if (new_door_open != door_open) {
		if (new_door_open)
			(*door_opened)();
		else if (!new_door_open)
			(*door_closed)();
		
		door_open = new_door_open;
	}
	
	
	// Monitor door-handle if door open
	unsigned long time = doorhandle_sense();
	unsigned long dev  = (unsigned long)abs(((int)doorhandle_avg) - ((int)time));
	
	// If there is a significant jump upward in time then someone just touched it!
	bool fire_event = false;
	if (time > doorhandle_avg * DOORHANDLE_SENSITIVITY) {
		fire_event = true;
		doorhandle_avg = time;
	} else {
		// Update the average time slowly
		doorhandle_avg = ((doorhandle_avg << DOORHANDLE_AVG_DECAY) - doorhandle_avg + time)
		                 >> DOORHANDLE_AVG_DECAY;
	}
	
	if (fire_event && (millis() - last_door_handle_touch > DOORHANDLE_MIN_PERIOD)) {
		(*door_handle_touched)();
		last_door_handle_touch = millis();
	}
}



/******************************************************************************
 * Mainloop/Initialisation                                                    *
 ******************************************************************************/

void
setup()
{
	shetsource_init();
	lights_init();
	pir_init();
	servo_init();
	touch_init();
	door_init();
	
	// Post connection
	shetsource_refresh();
	lights_post_connect();
}


void
loop()
{
	shetsource_refresh();
	lights_refresh();
	pir_refresh();
	servo_refresh();
	touch_refresh();
	door_refresh();
}

