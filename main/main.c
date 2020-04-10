#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <cpu_speed.h>

#include <graphics.h>
#include <sprite.h>
#include <macros.h>
#include <lcd_model.h>
#include <ascii_font.h>

#define W 7
#define H 6
#define NUM_OF_OBSTACLES 5
#define OVERFLOW_TOP 100
#define SPEED_MAX 100
#define DISTANCE_TO_WIN 10

#define sprite_move_to(sprite,_x,_y) (sprite.x = (_x), sprite.y = (_y))


uint8_t car_bm[] = {
	0b00111000,
	0b01111100,
	0b11000110,
	0b01111100,
	0b11101110,
	0b01111100
};

uint8_t fuel_bm[] = {
	0b01111110,
	0b10000001,
	0b10111101,
	0b10000001,
	0b10000001,
	0b10000001,
	0b11111111
};

uint8_t house_bm[] = {
	0b00010000,
	0b00111000,   // 7 wide, 7 high
	0b01111100,
	0b11111110,
	0b01000100,
	0b01010100,
	0b01111100
};

uint8_t tree_bm[] = {
	0b00111000,
	0b01111110, //// 8 wide, 8 high
	0b11111111,
	0b00111100,
	0b00011000,
	0b00011000,
	0b00011000,
	0b00011000
};

uint8_t cone_bm[] = {
	0b00010000,
	0b00101000,
	0b01111100,    /// 7 wide, 5 high
	0b01000100,
	0b11111110,
};

uint8_t zombie_bm[] = {
	0b10100101,
	0b11111111,
	0b10000001,
	0b10100101,   //// 8 wide, 7 high
	0b10000001,
	0b10001001,
	0b01111110
};

uint8_t heart_icon[8] = {
	0b11111111,
	0b00000000,
	0b00000000,
	0b01100110,
	0b11111111,
	0b01111110,
	0b00111100,
	0b00011000
};

uint8_t heart_direct[8];

uint8_t fuel_icon[8] = {
	0b11111111,
	0b00000000,
	0b00000000,
	0b01111100,
	0b10000010,
	0b10111010,
	0b10000010,
	0b11111110,
};

uint8_t fuel_direct[8];

uint8_t speed_icon[8] = {
	0b11111111,
	0b00000000,
	0b00000000,
	0b00011000,
	0b00100100,
	0b01010010,
	0b10100001,
	0b11111111,
};

uint8_t speed_direct[8];


volatile float speed;
float dy;
float distance;
float real_distance;
volatile float fuel; // initialise to 100
float fuel_increment; // 0 in setup
uint8_t health;
bool fuel_increment_set; // false in setup
bool car_fuel_pos; // false in setup()

Sprite fuel_station;
Sprite car;

Sprite objects[5];


bool paused; // initialised to false in setup
bool splash_shown; // initialised to true in setup()
bool fuel_on_screen; // initialised to false in setup.
volatile bool game_over; // false in setup
bool game_won; /// false in setup
bool distance_reached; // false in setup

volatile uint32_t timer_counter;
volatile uint8_t five_seconds_counter;

#define NUMBER_OF_BUTTONS 7
uint8_t bit_set[NUMBER_OF_BUTTONS];
uint8_t button_pressed[NUMBER_OF_BUTTONS];
#define JOY_UP 0
#define JOY_DOWN 1
#define JOY_LEFT 2
#define JOY_RIGHT 3
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 5
#define JOY_CENTRE 6


void adc_init() {
	// ADC Enable and pre-scaler of 128: ref table 24-5 in datasheet
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
	// Select AVcc voltage reference and pin combination.
	// Low 5 bits of channel spec go in ADMUX(MUX4:0)
	// 5th bit of channel spec goes in ADCSRB(MUX5).
	ADMUX = (channel & ((1 << 5) - 1)) | (1 << REFS0);
	ADCSRB = (channel & (1 << 5));

	// Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);

	// Wait for ADSC bit to clear, signalling conversion complete.
	while ( ADCSRA & (1 << ADSC) ) {}

	// Result now available.
	return ADC;
}


void new_lcd_init(uint8_t contrast) {
    // Set up the pins connected to the LCD as outputs
    SET_OUTPUT(DDRD, SCEPIN); // Chip select -- when low, tells LCD we're sending data
    SET_OUTPUT(DDRB, RSTPIN); // Chip Reset
    SET_OUTPUT(DDRB, DCPIN);  // Data / Command selector
    SET_OUTPUT(DDRB, DINPIN); // Data input to LCD
    SET_OUTPUT(DDRF, SCKPIN); // Clock input to LCD

    CLEAR_BIT(PORTB, RSTPIN); // Reset LCD
    SET_BIT(PORTD, SCEPIN);   // Tell LCD we're not sending data.
    SET_BIT(PORTB, RSTPIN);   // Stop resetting LCD

    LCD_CMD(lcd_set_function, lcd_instr_extended);
    LCD_CMD(lcd_set_contrast, contrast);
    LCD_CMD(lcd_set_temp_coeff, 0);
    LCD_CMD(lcd_set_bias, 3);

    LCD_CMD(lcd_set_function, lcd_instr_basic);
    LCD_CMD(lcd_set_display_mode, lcd_display_normal);
    LCD_CMD(lcd_set_x_addr, 0);
    LCD_CMD(lcd_set_y_addr, 0);
}

void setup_heart(void) {
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            uint8_t bit_val = BIT_VALUE(heart_icon[j], (7 - i));
            WRITE_BIT(heart_direct[i], j, bit_val);
        }
    }
}

void draw_heart(void) {
	uint8_t heart_x = 3;
	uint8_t heart_y = 4;

  LCD_CMD(lcd_set_function, lcd_instr_basic | lcd_addr_horizontal);
  LCD_CMD(lcd_set_x_addr, heart_x);
  LCD_CMD(lcd_set_y_addr, heart_y / 7);

  for (int i = 0; i < 8; i++) {
      LCD_DATA(heart_direct[i]);
  }
}

void setup_fuel(void) {
	for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {
					uint8_t bit_val = BIT_VALUE(fuel_icon[j], (7 - i));
					WRITE_BIT(fuel_direct[i], j, bit_val);
			}
	}
}

void draw_fuel(void) {
	uint8_t fuel_x = 34;
	uint8_t fuel_y = 4;

  LCD_CMD(lcd_set_function, lcd_instr_basic | lcd_addr_horizontal);
  LCD_CMD(lcd_set_x_addr, fuel_x);
  LCD_CMD(lcd_set_y_addr, fuel_y / 7);

  for (int i = 0; i < 8; i++) {
      LCD_DATA(fuel_direct[i]);
  }
}

void setup_speed(void) {
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            uint8_t bit_val = BIT_VALUE(speed_icon[j], (7 - i));
            WRITE_BIT(speed_direct[i], j, bit_val);
        }
    }

}

void draw_speed(void) {
	uint8_t speed_x = 63;
	uint8_t speed_y = 4;

  LCD_CMD(lcd_set_function, lcd_instr_basic | lcd_addr_horizontal);
  LCD_CMD(lcd_set_x_addr, speed_x);
	LCD_CMD(lcd_set_y_addr, speed_y / 8);

  for (int i = 0; i < 8; i++) {
      LCD_DATA(speed_direct[i]);
	}
}

void setup_icons(void) {
	lcd_clear();
	new_lcd_init(LCD_DEFAULT_CONTRAST);

	setup_heart();
	setup_fuel();
	setup_speed();
}

void draw_icons(void) {
	draw_heart();
	draw_fuel();
	draw_speed();

}

void draw_formatted(uint8_t x, uint8_t y, colour_t colour, const char * format, ...) {
	va_list args;
	va_start(args, format);
	char buffer[1000];
	vsprintf(buffer, format, args);
	draw_string(x, y, buffer, colour);
}

void setup_car(void) {
	int x = LCD_X / 2 - W / 2;
	int y = 3 * LCD_Y / 4 - 1;
	sprite_init(&car, x, y, W, H, car_bm);
}

void step_car(void) {

	int right_adc = adc_read(1);

	if (car.x > 0 && right_adc < 512) {
		car.x -= speed * 2 * ((512 - right_adc) / 512);

	}
	if (car.x < LCD_X - W && right_adc > 512) {
		car.x += speed * 2 * (right_adc / 512);
	}

	draw_formatted(20, 20, FG_COLOUR, "%d", right_adc);

	/*
  // left
  if (button_pressed[JOY_LEFT] == 1) { ///// if (BIT_IS_SET(PINB, 1)) {
    if (car.x > 0) {
      car.x -= 1; ///  speed * 0.2
    }
  }

  // right
  else if (button_pressed[JOY_RIGHT]) { //// else if (BIT_IS_SET(PIND, 0)) {
    if (car.x < (LCD_X - W)) {
      car.x +=  1; /// speed * 0.2
    }
  }
	*/


}

void draw_road(void) {
	double road_equation;
	for (int x = 0; x < LCD_Y; x++) {
		road_equation = 5*sin((x - (distance))/3) * 1.55 *sin((x - (distance))/ 7) * 2 * sin((x - distance) / 20); /// bunch of sin(x - distance) stuff

		draw_pixel(road_equation + 25, x, FG_COLOUR);
		draw_pixel(road_equation + 59, x, FG_COLOUR);
	}

}
////
bool offroad(void) {
	uint8_t left_pixel;
	uint8_t right_pixel;
	for (int i = 0; i < H; i++ ) {
		int x = (3 * LCD_Y / 4) + i;

		left_pixel =  5*sin((x - (distance))/3) * 1.55 *sin((x - (distance))/ 7) * 2 * sin((x - distance) / 20) + 25; /////
		right_pixel =  5*sin((x - (distance))/3) * 1.55 *sin((x - (distance))/ 7) * 2 * sin((x - distance) / 20) + 59; ////

		//// USE SCREEN BUFFER !!!
		// check left of car
		if (car.x <= left_pixel || car.x + W - 1 >= right_pixel) {
			if (speed > 3) {
				speed = 3;
			}
			return true;
		}

		// check right of car

	}

	return false;

}
///
void create_obstacle(int x, int y, int index) {
	int zombie = rand() % 2;

	if (zombie) {
		sprite_init(&objects[index], x, y, 8, 7, zombie_bm);
	}
	else {
		sprite_init(&objects[index], x, y, 7, 5, cone_bm);
	}
}
////
void create_scenery(int x, int y, int index) {
	int house = rand() % 2;

	if (house) {
		sprite_init(&objects[index], x, y, 7, 7, house_bm);
	}
	else {
		sprite_init(&objects[index], x, y, 8, 8, tree_bm);
	}
}
/////
void create_object(bool setup, int index) {
	int object_x = rand() % (LCD_X - 8 - 1);
	int object_y;
	if (setup) {
		object_y = rand() % (LCD_Y - 8 - 1) * - 1;
	}
	else {
		object_y = 0;
	}

	int left_pixel = 5*sin((object_y - distance)/3) * 1.55 *sin((object_y - distance)/ 7) * 2 * sin((object_y - distance)/ 20) + 25;
	int right_pixel = 5*sin((object_y - distance)/3) * 1.55 * sin((object_y - distance)/ 7) * 2 * sin((object_y - distance) / 20) + 59;


	if (object_x < right_pixel && object_x > left_pixel) {
		for (int y = object_y; y < object_y + 8; y++) {
			int check_left = 5*sin((y - distance)/3) * 1.55 *sin((y - distance)/ 7) * 2 * sin((y - distance)/ 20) + 25;
			int check_right = 5*sin((y - distance)/3) * 1.55 * sin((y - distance)/ 7) * 2 * sin((y - distance) / 20) + 59;

			if (object_x < check_left) {
				int difference = check_left - object_x;
				object_x += difference;
			}
			else if (object_x + 7 /* + 8 - 1*/ > check_right) {
				int difference = object_x + 7 - check_right;
				object_x -= difference;
			}

		}

		create_obstacle(object_x, object_y, index);
	}

	else {

		for (int y = object_y; y < object_y + 8; y++) {
			int check_left = 5*sin((y - distance)/3) * 1.55 *sin((y - distance)/ 7) * 2 * sin((y - distance)/ 20) + 25;
			int check_right = 5*sin((y - distance)/3) * 1.55 * sin((y - distance)/ 7) * 2 * sin((y - distance) / 20) + 59;

			if (object_x < check_right && object_x > check_left) {
				int difference = check_right - object_x;
				object_x += difference;
			}
			else if (object_x + 7 > check_left && object_x < check_right) {
				int difference = object_x + 7 - check_left;
				object_x -= difference;
			}
			else if (object_x + 7 > LCD_X - 1) {
				int difference = object_x + 7 - LCD_X - 1;
				object_x -= difference;
			}

		}

		if (object_x < 0) {
			object_x = 0;
		}

		create_scenery(object_x, object_y, index);
	}

}
////
bool space_occupied(Sprite first, Sprite second) {
	int first_left = (int) first.x;;
	int first_right = first_left + first.width - 1;;
	int first_top = (int) first.y;
	int first_bottom = first_top + first.height - 1;

	// sprite ur checking
	int second_left = (int) second.x;;
	int second_right = second_left + second.width - 1;;
	int second_top = (int) second.y;
	int second_bottom = second_top + second.height - 1;

	// collision comparisons
	if (first_top > second_bottom) return false;
	if (second_top > first_bottom) return false;
	if (first_left > second_right) return false;
	if (second_left > first_right) return false;

	return true;

}
////
bool test_overlap(void) {
	for (int i = 0; i < NUM_OF_OBSTACLES; i++) {
		for (int j = 0; j < NUM_OF_OBSTACLES; j++) {
			if ( i != j ) {
				bool overlap_present = space_occupied(objects[i], objects[j]);
				if (overlap_present) {
					return true;
				}
			}
		}
	}
	return false;
}
/////
void remove_overlap(bool setup) {
	for (int i = 0; i < NUM_OF_OBSTACLES; i++) {
		for (int j = 0; j < NUM_OF_OBSTACLES; j++) {
			if (i != j) {
				bool occupied = space_occupied(objects[i], objects[j]);
				while (occupied) {
					create_object(setup, i);
					occupied = space_occupied(objects[i], objects[j]);
				}
			}
		}
	}
}
/////
void step_sprite(int index) {
	if ((objects[index].y > LCD_Y) && !distance_reached) {
		create_object(false, index);

		bool overlap_present = test_overlap();

		while(overlap_present) {
			remove_overlap(false);
			overlap_present = test_overlap();
		}
	}

	else {
		objects[index].y += dy * 2;
	}

	sprite_draw(&objects[index]);
}

void debounce(void) {
	uint8_t mask = 0b11111111;
	for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
		bit_set[i] = bit_set[i] << 1; /////////????
	}

	bit_set[JOY_UP] |= BIT_IS_SET(PIND, 1);
	bit_set[JOY_DOWN] |=	BIT_IS_SET(PINB, 7);
	bit_set[JOY_LEFT] |= BIT_IS_SET(PINB, 1);
	bit_set[JOY_RIGHT] |= BIT_IS_SET(PIND, 0);
	bit_set[LEFT_BUTTON] |= BIT_IS_SET(PINF, 6);
	bit_set[RIGHT_BUTTON] |= BIT_IS_SET(PINF, 5);
	bit_set[JOY_CENTRE] |= BIT_IS_SET(PINB, 0);

	for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
		if (bit_set[i] == mask) {
			button_pressed[i] = 1;
		}
		else if (bit_set[i] == 0) {
			button_pressed[i] = 0;
		}
	}
}

void accelerate(void) {
	if ( button_pressed[JOY_DOWN] && speed > 0) {
		/// accelerator has no effect on car
		// decrease speed so that it would go from 10 to 0 in 2 seconds
		speed -= 0.32768; ///  0.32768

		if (speed < 0.32768) {
			speed = 0;
		}
	}


	if ( !offroad() ) {

		if (button_pressed[JOY_UP] && speed < 10  ) {
			/// if ( !offroad() )
			/// speed increases 1 to 10 in 5s
			speed += 0.1176948;
		}

		if ( !button_pressed[JOY_UP] && !button_pressed[JOY_DOWN]) {
			if (speed > 1 && speed > 0) {
				speed -= 0.196608;

				if (speed < 1.196608) {
					speed = 1;
				}
			}

			else if (speed < 1 && speed >= 0) {
				speed += 0.032768;

				if (speed > 0.96732) {
					speed = 1;
				}
			}
		}
	}

	else if ( offroad() ) {

		if (button_pressed[JOY_UP] && speed < 3) {
			speed += 0.0262144;
		}

		if ( !button_pressed[JOY_UP] && !button_pressed[JOY_DOWN]) {
			if (speed > 1 && speed > 0) {
				speed -= 0.0436907;

				if (speed < 1.0436906) {
					speed = 1;
				}

			}

			else if (speed < 1 && speed >= 0) {
				speed += 0.0218453;

				if (speed > 0.9781547) {
					speed = 1;
				}
			}
		}
	}

	dy = 0.05 * speed;

}

void update_distance(void) {
	distance += dy * 2;
	real_distance = distance / 30;
}

void update_fuel(void) {
	if (fuel > 0) {
		fuel -= 0.15 * dy;
	}
}

///
uint8_t station_x(void) {
	uint8_t orientation = rand() % 2;
	uint8_t a = 0;
	float x_pos;
	float compare;

	if (orientation == 1) { /// if on the left
		x_pos =  5*sin((a - (distance))/3) * 1.55 *sin((a - (distance))/ 7) * 2 * sin((a - distance) / 20) + 25;
		for (int x = 1; x < fuel_station.height; x++) {
			compare =  5*sin((x - (distance))/3) * 1.55 *sin((x - (distance))/ 7) * 2 * sin((x - distance) / 20) + 25;

			if (compare < x_pos) {
				x_pos = compare;
			}
		}

		return round(x_pos) - 8; // 8 for fuel station width

	}

	else { /// if on the right
		x_pos =  5*sin((a - (distance))/3) * 1.55 *sin((a - (distance))/ 7) * 2 * sin((a - distance) / 20) + 59;

		for (int x = 1; x < fuel_station.height; x++) {
			compare =  5*sin((x - (distance))/3) * 1.55 *sin((x - (distance))/ 7) * 2 * sin((x - distance) / 20) + 59;

			if (compare > x_pos) {
				x_pos = compare;
			}
		}

		return round(x_pos);
	}

}

void station_to_top(void) {
	fuel_station.x = station_x();
	fuel_station.y = 0;
}

void get_fuel_increment(void) {
	if ( !fuel_increment_set ) {
		float timer_period = 3.0 / 0.032768; /// timer overflow period
		if (speed == 0) {
			/// add fuel hm hm
			float fuel_difference = 100 - fuel;
			fuel_increment = fuel_difference / timer_period;
			fuel_increment_set = true;
		}
	}
}

void check_station(void) {
	if (fuel_on_screen) {
		uint8_t fuel_left = fuel_station.x;
		uint8_t fuel_right = fuel_left + 7;
		uint8_t fuel_top = (int) fuel_station.y;

		uint8_t car_left = car.x;
		uint8_t car_right = car_left + W - 1;
		uint8_t car_top = (int) car.y;

		if ((car_top >= fuel_top - 1) & (car_top <= fuel_top + 1)) {

			if (fuel_station.x < LCD_X / 2) {
				if ((car_left - 1 == fuel_right) | (car_left - 2 == fuel_right)) {
					car_fuel_pos = true;
					get_fuel_increment();
				}
			}

			else {
				if ((car_right == fuel_left - 1) | (car_right == fuel_left - 2)) {
					car_fuel_pos = true;
					get_fuel_increment();
				}

			}
		}

		if (fuel_station.y >= LCD_Y) {
			fuel_on_screen = false;
			fuel_increment_set = false;
			car_fuel_pos = false;
		}
	}


	if ( !fuel_on_screen ) {
		int chance_station_appears = 100 - fuel;
		double d_modulus_number = -0.4814 * fuel * fuel + 102.25 * fuel - 335.5; /// equation formed using trial and error and excel (shoutout excel)
		int modulus_number = (int)d_modulus_number;
		int percent = rand() % modulus_number;

		if (percent < chance_station_appears) {
				// make station show up
				station_to_top();
				fuel_on_screen = true;
		}
	}

}

void step_fuel(void) {
	if (fuel_on_screen) {
		fuel_station.y += dy * 2;
	}

	sprite_draw(&fuel_station);
}

int horizontal_overlap(Sprite object) {
  int overlap;

  int object_left = (int) object.x;
  int object_right = object_left + object.width - 1;

  int car_left = (int) car.x;
  int car_right = car_left + car.width - 1;

  if (object_right < car_left) return 0;
  if (object_left > car_right) return 0;

  if ((object_left <= car_left) || (object_right <= car_left)) {
    overlap = object_right - car_left + 1;

    if (overlap > object.width) {
      overlap = object.width;
    }
    else if (overlap > 7) {
      overlap = 7;
    }

  }

  else if (object_left <= car_right || object_left >= car_left) {
    overlap = car_right - object_left + 1;

    if (overlap > object.width) {
      overlap = object.width;
    }
    else if (overlap > 7) {
      overlap = 7;
    }

  }

  return overlap;
}

int vertical_overlap(Sprite object) {
  int object_top = round(object.y);
  int object_bottom = object_top + object.height - 1;
  int car_top = (int) car.y;
  int overlap;

  if (object_bottom >= car_top) {
    if (car_top >= object_top) {
      // car is lower than object
      overlap = object_bottom - car_top + 1;

      if (overlap > 6) {
        overlap = 6;
      }

      return overlap;
    }

    else { // if (object_top > car.y)
      overlap = 6 - (object_top - car_top)  /*40 + 1*/; //// [car.y + H - 1] (LCD HEIGHT is 48, 3/4 down - 1 is 35)

      if (overlap > object.height) {
        overlap = object.height;
      }

      if (overlap < 0) {
        overlap = 0;
      }

      return overlap;
    }

  }

  overlap = 0;
  return overlap;
}

bool check_collision(Sprite object) {
  int object_top =  round(object.y); /// (int) object.y;
  int object_bottom = object_top + object.height - 1;
  int object_left = round(object.x); //// (int) object.x;
  int object_right = object_left + object.width - 1;

  int car_top = round(car.y);
  int car_bottom = car_top + car.height - 1;
  int car_left = (int) car.x;
  int car_right = car_left + car.width - 1;

  if (object_top > car_bottom) return false;
  if (object_bottom < car_top) return false;
  if (object_right < car_left) return false;
  if (object_left > car_right) return false;


  uint8_t left_start = 0; //////
  uint8_t right_start = 0; //// ints v uint8_t

  if ((object_bottom >= car_top || object_top <= car_bottom) && vertical_overlap(object) > 0) {
    uint8_t right_bm;
    uint8_t left_bm;
    uint16_t word; /// werd nerd fly like a berd

    Sprite right = car; // assume car is on right
    Sprite left = object;  // assume object is on left

    if (car_left < object_left) {
      left = car;
      right = object;
    }

    /// set starts for left and right sprites
    if ((car_top < object_top && car_left < object_left) || (car_top > object_top && car_left > object_left)) {
      right_start = 0;
      left_start = left.height - vertical_overlap(object);
    }

    else if ((car_top > object_top && car_left < object_left) || (car_top < object_top && car_left > object_left)) {
      left_start = 0;
      right_start = right.height - vertical_overlap(object);
    }



    for (int i = 0; i < vertical_overlap(object); i++) {

      right_bm = (right.bitmap[right_start + i] >> (8 - right.width));
      left_bm = (left.bitmap[left_start + i] >> (8 - left.width));

      int word_length = right.width + left.width - horizontal_overlap(object); /// find HORIZONTAL OVERLAP
      int left_shift = word_length - left.width;

      word = left_bm << left_shift;
      word &= right_bm;


      if (word != 0) {
				draw_formatted(20, 20, FG_COLOUR,":(");
        return true;
      }

    }

    }


  return false;

}

void relocate_car(void) {
	int x = rand() % (LCD_X - W - 1);
	car.x = x;

	bool overlap = false;
	bool station_overlap = false;


	if (fuel_on_screen) {
		station_overlap = space_occupied(fuel_station, car);
	}


	for (int i = 0; i < NUM_OF_OBSTACLES; i++) {
		overlap = space_occupied(car, objects[i]);
		while (overlap || station_overlap) {
			x = rand() % (LCD_X - W - 1);
			car.x = x;

			station_overlap = space_occupied(fuel_station, car);
			overlap = space_occupied(objects[i], car);
		}
	}

	speed = 0;
	fuel = 100;

}

void inflict_damage(void) {
	if (speed < 3) {
			health -= 10;
	}
	else if (speed < 5) {
			health -= 20;
	}
	else if (speed < 7) {
			health -= 30;
	}
	else if (speed < 9) {
			health -= 40;
	}
	else {
			health -= 50;
	}


}

void check_game_over(void) {
	bool collision;

	for (int i = 0; i < NUM_OF_OBSTACLES; i++) {
		collision = check_collision(objects[i]);
		if (collision) {
			inflict_damage();

			if (health < 1) {
				game_over = true;
			}
			else {
				relocate_car();
			}
		}
	}

	bool fuel_collision = check_collision(fuel_station);
	if (fuel_collision) {
		game_over = true;
	}

	if (fuel < 1) {
		fuel = 0;
		game_over = true;
	}

}

double elapsed_time(void) {
	double time = (timer_counter * 65536.0 + TCNT1) / 8000000.0;
	return time;
}

void set_duty_cycle(int duty_cycle) {
	TC4H = duty_cycle >> 8;
	OCR4A = duty_cycle & 0xff;
}

void setup_backlight(void) {
	TC4H = OVERFLOW_TOP >> 8;
	OCR4C = OVERFLOW_TOP & 0xff;

	TCCR4A = (1 << COM4A1) | (1 << PWM4A);
	SET_BIT(DDRC, 7);

	TCCR4B = (1 << CS42) | (1 << CS41) | (1 << CS40);

	TCCR4D = 0;
}

void backlight_process(void) {
	if (splash_shown) {
		set_duty_cycle(0);
	}

	else {
		int duty_speed = speed * 10;
		set_duty_cycle(SPEED_MAX - duty_speed);
	}

}


ISR(TIMER0_OVF_vect) {
	check_game_over();

	if (speed == 0 && car_fuel_pos && fuel < 100) {
		fuel += fuel_increment;
	}

}

ISR(TIMER1_OVF_vect) {
	if ( !paused && !splash_shown ) {
		timer_counter++; /// move this into a different timeeeerrrrr
	}

		debounce();
}

ISR(TIMER3_OVF_vect) {
	if ( !paused  && !splash_shown) {
		accelerate();
	}

}

void setup_timers(void) {
	// timer 0 stuffs
	TCCR0A = 0;
	TCCR0B = 5; /// prescale to 1024, overflow period 0.032768s
	TIMSK0 = 1;

	// timer 1 stuffs
	TCCR1A = 0; // initialise timer
	TCCR1B = 1; // set overflow period to 0.008 or whatever it is set to lols
	TIMSK1 = 1; // enable timer overflow

	// timer 3 stuffs
	TCCR3A = 0; // initialise timer
	TCCR3B = 2; // set overflow period to 0.065536
	TIMSK3 = 1; // enable timer overflow

	timer_counter = 0;
	sei();
}


void draw_dashboard(void) {
	// draw partition
	/// clear stuffs
	for (int i = 0; i <= 10; i++) {
		draw_line(0, i, LCD_X - 1, i, BG_COLOUR);
	}

	// draw actual lines
	draw_line(0, 0, LCD_X - 1, 0, FG_COLOUR); // top
	draw_line(0, 0, 0, 10, FG_COLOUR); // left
	draw_line(LCD_X - 1, 0, LCD_X, 10, FG_COLOUR); // right
	draw_line(0, 10, LCD_X - 1, 10, FG_COLOUR); // bottom


	// draw speed
	// uint8_t int_speed = (int)speed;
	draw_formatted(12, 2, FG_COLOUR, "%.0d", health);

	// draw fuel
	draw_formatted(42, 2, FG_COLOUR, "%.0f", fuel);

	/// draw condition
	///// insert stuff

	// draw distance
	draw_formatted(73, 2, FG_COLOUR, "%d", (int) speed);

}

void display_splash(void) {
	while (splash_shown) {
		draw_string(0, 0, "pew pew pew pew", FG_COLOUR);
		draw_string(0, 11, "A YIZZLE", FG_COLOUR);
		draw_string(0, 22, "n9632590", FG_COLOUR);

		show_screen();

		if (button_pressed[LEFT_BUTTON] == 1 || button_pressed[RIGHT_BUTTON]) {
			splash_shown = false;
		}
	}
}

void pause_screen(double paused_time) {
	if (!splash_shown) {

		if (button_pressed[JOY_CENTRE]) {
			paused = true;
		}

		while (paused) {

			char * pause_message[6] = {
				"     PAUSED",
				"      ",
				"      ",
				"      ",
				"      ",
				"  [R] to resume"
			};

			for (int i = 0; i < 6; i++) {
				draw_formatted(0, 8 * i, FG_COLOUR, pause_message[i]);
			}

			draw_formatted(12, 16, FG_COLOUR, "Time: %.2fs", (float) paused_time);
			draw_formatted(8, 24, FG_COLOUR, "Distance: %dkm", (int) distance);
			show_screen();

			if (button_pressed[RIGHT_BUTTON]) {
				paused = false;
			}

		}

	}

}

bool sprites_visible(void) {
	for (int i = 0; i < NUM_OF_OBSTACLES; i++) {
		if (objects[i].y < LCD_Y) {
			return true;
		}
	}
	if (fuel_on_screen) {
		return true;
	}
	return false;
}

void victory_screen(void) {
	while (car.y > 0) {
		clear_screen();
		draw_road();
		set_duty_cycle(0);

		car.y -= 1;
		sprite_draw(&car);

		draw_dashboard();
		show_screen();
		draw_icons();
	}

}



void game_over_sequence(void) {
	clear_screen();

	// uint8_t message_length = 2;

	while (!button_pressed[LEFT_BUTTON]) {
		if (game_won) {
			/// victory message
				char * message[2] = {
					"     You won!     ",
					"       yay        ",
				};

				for (int i = 0; i < 2; i++) {
					draw_string(0, i * 8, message[i], FG_COLOUR);
				}
		}

		else {
			/// u lost lols
			char * message[2]  = {
				"     You lost!     ",
				"       lol         "
			};

			for (int i = 0; i < 2; i++) {
				draw_string(0, i * 8, message[i], FG_COLOUR);
			}
		}

		show_screen();
	}

}

void reset_variables(void) {
	game_over = false;
	game_won = false;
	distance_reached = false;
	speed = 0;
	distance = 0;
	real_distance = 0;
	paused = false;
	splash_shown = true;
	fuel = 100;
	fuel_on_screen = false;
	fuel_increment_set = false;
	fuel_increment = 0;
	car_fuel_pos = false;
	health = 100;
}

void setup(void) {
	set_clock_speed(CPU_8MHz);
	lcd_init(LCD_DEFAULT_CONTRAST);
	adc_init();

	lcd_clear();
	new_lcd_init(LCD_DEFAULT_CONTRAST);

	setup_backlight();

	clear_screen();

  // joystick stuffs
  CLEAR_BIT(DDRB, 1); // left
  CLEAR_BIT(DDRB, 7); // down
  CLEAR_BIT(DDRD, 0); // right
  CLEAR_BIT(DDRD, 1); // up
	CLEAR_BIT(DDRB, 0); // centre
	CLEAR_BIT(DDRF, 6); /// button left
	CLEAR_BIT(DDRF, 5); /// button right

	setup_timers();


	// set errythang to 0
	reset_variables();
	sprite_init(&fuel_station, 0, 0, 8, 7, fuel_bm);

	// seed random
	//// srand(car.x); /// idk what to seed dis tooooo

	for (int i = 0; i < NUM_OF_OBSTACLES; i++) {
		create_object(true, i);
	}

	bool overlap_present = test_overlap();
	while (overlap_present) {
		remove_overlap(true);
		overlap_present = test_overlap();
	}

	/// setup icons
	setup_icons();

	setup_car();

	// show_screen(); //// don't show screen bc
}

void process(void) {
	clear_screen();

	pause_screen(elapsed_time());

	draw_road();
	backlight_process();

	/// car stuff
	/// change_speed();
	step_car();
	sprite_draw(&car);
	offroad();

	/// object stuff
	for (int i = 0; i < NUM_OF_OBSTACLES; i++) {
		step_sprite(i);
	}

	check_station();
	step_fuel();

	update_distance();
	update_fuel();
	draw_dashboard();

	show_screen();

	draw_icons();

	if (distance >= DISTANCE_TO_WIN) {
		distance_reached = true;
	}
	if (!sprites_visible()) {
		game_won = true;
	}

}

int main(void) {
	bool program_running = true;

	while (program_running) {
		setup();
		clear_screen();
		display_splash();

		while ( !game_over ) {
			process();
			_delay_ms(10);


			if (game_won) {
				game_over = true;
			}

		}

		if (game_won) {
			victory_screen();
		}

		game_over_sequence();
	}
}
