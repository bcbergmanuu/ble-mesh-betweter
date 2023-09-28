#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

LOG_MODULE_REGISTER(shiftregister, LOG_LEVEL_DBG);

struct gpio_dt_spec specs[] = {
	DT_FOREACH_PROP_ELEM_SEP(ZEPHYR_USER_NODE, relais_gpios,
							 GPIO_DT_SPEC_GET_BY_IDX, (, ))};

static uint8_t numbers_to_display[10] = {219,144,93,213,150,199,207,145,223,215};

static uint8_t numbermask(uint8_t number, bool dot) {
  return numbers_to_display[number] + (dot ? 32 :0);
}

static void flush_shift_register()
{
  gpio_pin_set_dt(&specs[1], 0);  
  k_sleep(K_USEC(1));
  gpio_pin_set_dt(&specs[1], 1);
  k_sleep(K_USEC(1));
}

static void write_bit(bool b)
{
  gpio_pin_set_dt(&specs[0], !b);
  k_sleep(K_USEC(1));
  gpio_pin_set_dt(&specs[2], 0);  
  k_sleep(K_USEC(1));
  gpio_pin_set_dt(&specs[0], b);
  k_sleep(K_USEC(1));
  gpio_pin_set_dt(&specs[2], 1);  
  k_sleep(K_USEC(1));
}

static void write_bitmask(uint8_t btm)
{
  int b = 1;
  for(int x=0; x<8; x++) {
      (b & btm) ? write_bit(1) : write_bit(0);
      b <<=1;
  }
}

static uint16_t number = 0;
static void drawnumber_work_handler(struct k_work *work) {
  
  uint8_t d_seconds, d_seconds_l, d_seconds_h, d_minutes = 0;
  
  d_minutes = number/60;
  d_seconds = number- (d_minutes*60);
  d_seconds_l = (d_seconds % 10);
  d_seconds_h = (d_seconds % 100-d_seconds_l)/10;
  number++;
    
  write_bitmask(numbermask(d_seconds_l, 0));
  write_bitmask(numbermask(d_seconds_h, 0));
  write_bitmask(numbermask(d_minutes, 1));

  flush_shift_register();
  printk("drawnumber: %d\n", number);
}

K_WORK_DEFINE(clockworker, drawnumber_work_handler);

void drawnumber(struct k_timer *timer_id) {
  k_work_submit(&clockworker);  
}

K_TIMER_DEFINE(clocktimer, drawnumber, NULL);

//serial = specs_0; green
//sck = specs_1     white
//srck = specs_2    yellow

//high out = specs_3

void stop_clock() {
  k_timer_stop(&clocktimer);
  printk("light off");
  printk("clock stopped\n");
}

void start_clock() {
  number = 0;
  printk("light on");
  k_timer_start(&clocktimer, K_SECONDS(1), K_SECONDS(1));
}

// set all pins as output
int init_clock()
{	
  int err = 0;
  
	for (size_t x = 0; x < ARRAY_SIZE(specs); x++)
	{
		err = gpio_pin_configure_dt(&specs[x], GPIO_OUTPUT_INACTIVE);

		if (err)
		{
			LOG_ERR("Cannot configure LED gpio");
			return err;
		}
	}

  k_timer_start(&clocktimer, K_SECONDS(1), K_SECONDS(1));	
	return 0;
}