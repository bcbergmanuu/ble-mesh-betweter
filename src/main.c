/*
 * The unprovisioned beacon uses the device address set by Nordic
 * in the FICR as its UUID and is presumed unique.
 *
 * Button and LED 1 are in the root node.
 * The 3 remaining button/LED pairs are in element 1 through 3.
 * Assuming the provisioner assigns 0x100 to the root node,
 * the secondary elements will appear at 0x101, 0x102 and 0x103.
 *
 * It's anticipated that after provisioning, the button clients would
 * be configured to publish and the LED servers to subscribe.
 *
 * If a LED server is provided with a publish address, it will
 * also publish its status on a state change.
 *
 * Messages from a button to its corresponding LED are ignored as
 * the LED's state has already been changed locally by the button client.
 *
 *
 */

#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/mesh.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
#include "shiftregister.h"

// #ifdef CONFIG_USB_DEVICE_STACK
// #include <zephyr/drivers/uart.h>
// #include <zephyr/usb/usb_device.h>
// #endif

LOG_MODULE_REGISTER(onoffapp, LOG_LEVEL_DBG);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
struct gpio_dt_spec flashlight = GPIO_DT_SPEC_GET_OR (ZEPHYR_USER_NODE, flashlight_gpios, {0});

/* Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)

static int gen_onoff_set(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf);

static int gen_onoff_set_unack(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf);

static int gen_onoff_get(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf);

static int gen_onoff_status(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf);

/*
 * Client Configuration Declaration
 */
static bool attention;
static struct bt_mesh_cfg_cli cfg_cli = {
};


/*
 * Publication Declarations
 *
 * The publication messages are initialized to the
 * the size of the opcode + content
 *
 * For publication, the message must be in static or global as
 * it is re-transmitted several times. This occurs
 * after the function that called bt_mesh_model_publish() has
 * exited and the stack is no longer valid.
 *
 * Note that the additional 4 bytes for the AppMIC is not needed
 * because it is added to a stack variable at the time a
 * transmission occurs.
 *
 */

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv_s_0, NULL, 2 + 2);
//BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli_s_0, NULL, 2 + 2);
//BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv_s_1, NULL, 2 + 2);
//BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli_s_1, NULL, 2 + 2);
//BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv_s_2, NULL, 2 + 2);
//BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli_s_2, NULL, 2 + 2);

/*
 * Models in an element must have unique op codes.
 *
 * The mesh stack dispatches a message to the first model in an element
 * that is also bound to an app key and supports the op code in the
 * received message.
 *
 */

/*
 * OnOff Model Server Op Dispatch Table
 *
 */

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET,       BT_MESH_LEN_EXACT(2), gen_onoff_set },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, BT_MESH_LEN_EXACT(2), gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

/*
 * OnOff Model Client Op Dispatch Table
 */

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, BT_MESH_LEN_EXACT(1), gen_onoff_status },
	BT_MESH_MODEL_OP_END,
};

struct onoff_state {
	const struct gpio_dt_spec led_device;
	uint8_t server_num;
	int8_t current;
	//uint8_t previous;
	uint8_t istriggered;
};

/*
 * Declare and Initialize Element Contexts
 * Change to select different GPIO output pins
 */

static struct onoff_state onoff_state[] = {
	{ .led_device = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), .server_num = 0,},
	{ .led_device = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios), .server_num= 1,},
	{ .led_device = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios), .server_num= 2,},
	{ .led_device = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios), .server_num= 3,},
};

/*
 * Health Server Declaration
 */
//static struct k_work_delayable attention_blink_work;

static void attention_blink(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(attention_blink_work, attention_blink);
static void attention_blink(struct k_work *work)
{	
	static int idx =0;
	struct onoff_state health_led = onoff_state[1];
	if (attention) {				
		gpio_pin_set_dt(&health_led.led_device, (idx++)%2);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		gpio_pin_set_dt(&health_led.led_device, 0);
	}
	
	k_work_reschedule(&attention_blink_work, K_MSEC(30));
}



static void attention_on(struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(struct bt_mesh_model *mod)
{
	/* Will stop rescheduling blink timer */
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};


/*
 *
 * Element Model Declarations
 *
 * Element 0 Root Models
 */

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv, &onoff_state[0]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
		      &gen_onoff_pub_cli, &onoff_state[0]),
};

/*
 * Element 1 Models
 */

static struct bt_mesh_model secondary_0_models[] = {
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv_s_0, &onoff_state[1]),
	// BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
	// 	      &gen_onoff_pub_cli_s_0, &onoff_state[1]),
};

/*
 * Element 2 Models
 */

// static struct bt_mesh_model secondary_1_models[] = {
// 	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
// 		      &gen_onoff_pub_srv_s_1, &onoff_state[2]),
// 	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
// 		      &gen_onoff_pub_cli_s_1, &onoff_state[2]),
// };

/*
 * Element 3 Models
 */

// static struct bt_mesh_model secondary_2_models[] = {
// 	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
// 		      &gen_onoff_pub_srv_s_2, &onoff_state[3]),
// 	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
// 		      &gen_onoff_pub_cli_s_2, &onoff_state[3]),
// };

/*
 * Button to Client Model Assignments
 */

struct bt_mesh_model *mod_cli_sw[] = {
		&root_models[4],
		// &secondary_0_models[1],
		// &secondary_1_models[1],
		// &secondary_2_models[1],
};

/*
 * LED to Server Model Assignments
 */

struct bt_mesh_model *mod_srv_sw[] = {
		&root_models[3],
		&secondary_0_models[0],
		// &secondary_1_models[0],
		// &secondary_2_models[0],
};

/*
 * Root and Secondary Element Declarations
 */

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
	BT_MESH_ELEM(0, secondary_0_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};


//static uint8_t trans_id;
static uint16_t primary_addr;
static uint16_t primary_net_idx;

/*
 * Generic OnOff Model Server Message Handlers
 *
 * Mesh Model Specification 3.1.1
 *
 */

static int gen_onoff_get(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	//thing this does nothing 
	//return 0;
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
	struct onoff_state *onoff_state = model->user_data;

	printk("addr 0x%04x onoff 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, onoff_state->current);
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
	net_buf_simple_add_u8(&msg, onoff_state->current);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send On Off Status response\n");
	}

	return 0;
}


static uint8_t currentstate = 2;
static void state_update_delaywork(struct k_work *work) {
	if(onoff_state[0].current == 1 && onoff_state[1].current == 1)
	onoff_state[0].istriggered = 0;
	onoff_state[1].istriggered = 0;		
	currentstate = 3;
	printk("new delayed state %i\n", currentstate);
}

K_WORK_DELAYABLE_DEFINE(stateupdate, state_update_delaywork);


static void gamestate() {
	
	printk("current state %i\n", currentstate);
	
	if(currentstate == 0) { //running state
		
		if(onoff_state[0].istriggered && onoff_state[1].istriggered) {						
			stop_clock();
			gpio_pin_set_dt(&flashlight, 1);  //turn on flashlight
			currentstate = 1; //two buttons have been pressed
		}			
	
	} else if(currentstate == 1) {
		if(onoff_state[0].current == 0 && onoff_state[1].current == 0) {							
			currentstate = 2; //both buttons are released
		}
	
	} else if(currentstate == 2) {		
		if(onoff_state[0].current == 1 && onoff_state[1].current == 1) {						
			k_work_reschedule(&stateupdate, K_SECONDS(1));
		}	//both buttons are pressed again to start the game
	
	} else if(currentstate == 3) { //trigger set to zero, when ether button is released the clock starts
		if(onoff_state[0].current == 0 || onoff_state[1].current == 0) {						
			start_clock();
			gpio_pin_set_dt(&flashlight, 0);  //turn off flashlight
			currentstate = 0;
		}
	}
	printk("new state %i\n", currentstate);
}

static int gen_onoff_set_unack(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	//struct net_buf_simple *msg = model->pub->msg;
	struct onoff_state *onoff_state = model->user_data;
	

	onoff_state->current = net_buf_simple_pull_u8(buf);
	printk("server %i, addr 0x%02x state 0x%02x\n", onoff_state->server_num, bt_mesh_model_elem(model)->addr, onoff_state->current);

	gpio_pin_set_dt(&onoff_state->led_device, onoff_state->current);
	
	if(onoff_state->current){
		onoff_state->istriggered = 1;
	}
	
	gamestate();
	/*
	 * If a server has a publish address, it is required to
	 * publish status on a state change
	 *
	 * See Mesh Profile Specification 3.7.6.1.2
	 *
	 * Only publish if there is an assigned address
	 */

	// if (onoff_state->previous != onoff_state->current &&
	//     model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
	// 	printk("publish last 0x%02x cur 0x%02x\n",
	// 	       onoff_state->previous, onoff_state->current);
	// 	onoff_state->previous = onoff_state->current;
	// 	bt_mesh_model_msg_init(msg,
	// 			       BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
	// 	net_buf_simple_add_u8(msg, onoff_state->current);
	// 	err = bt_mesh_model_publish(model);
	// 	if (err) {
	// 		printk("bt_mesh_model_publish err %d\n", err);
	// 	}
	// }

	return 0;
}

static int gen_onoff_set(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	printk("gen_onoff_set\n");

	(void)gen_onoff_set_unack(model, ctx, buf);
	(void)gen_onoff_get(model, ctx, buf);

	return 0;
}

static int gen_onoff_status(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{
	uint8_t	state;

	state = net_buf_simple_pull_u8(buf);

	printk("Node 0x%04x OnOff status from 0x%04x with state 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, ctx->addr, state);

	return 0;
}

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number %06u\n", number);
	return 0;
}

static int output_string(const char *str)
{
	printk("OOB String %s\n", str);
	return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	printk("provisioning complete for net_idx 0x%04x addr 0x%04x\n",
	       net_idx, addr);
	primary_addr = addr;
	primary_net_idx = net_idx;
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

struct buttonwork {
    struct k_work work;
    uint8_t button_num;
	uint8_t button_pressed;
};

static uint8_t trans_id = 0;

static void button_pressed_worker(struct k_work *workitem)
{
	printk("button_pressed_worker\n");
	struct buttonwork *button = CONTAINER_OF(workitem, struct buttonwork, work);

	//only to test gamestate;
	if(button->button_num == 1) {
		
		onoff_state[0].current = button->button_pressed;
		if(button->button_pressed) {
			onoff_state[0].istriggered =1;
		};
			
		gamestate();
		return;
	} if (button->button_num ==2) {
		onoff_state[1].current = button->button_pressed;
		if(button->button_pressed) {
			onoff_state[1].istriggered =1;
		};
		gamestate();
		return;
	}

	struct bt_mesh_model *mod_cli; // *mod_srv;
	struct bt_mesh_model_pub *pub_cli; // *pub_srv;
	
	int err;
	

	mod_cli = mod_cli_sw[0];
	pub_cli = mod_cli->pub;

	// mod_srv = mod_srv_sw[0];
	// pub_srv = mod_srv->pub;

	/* If unprovisioned, just call the set function.
	 * The intent is to have switch-like behavior
	 * prior to provisioning. Once provisioned,
	 * the button and its corresponding led are no longer
	 * associated and act independently. So, if a button is to
	 * control its associated led after provisioning, the button
	 * must be configured to either publish to the led's unicast
	 * address or a group to which the led is subscribed.
	 */

	// if (primary_addr == BT_MESH_ADDR_UNASSIGNED) {
	// 	NET_BUF_SIMPLE_DEFINE(msg, 1);
	// 	struct bt_mesh_msg_ctx ctx = {
	// 		.addr = 0 + primary_addr,
	// 	};

		/* This is a dummy message sufficient
		 * for the led server
		 */

		// net_buf_simple_add_u8(&msg, sw->onoff_state);
		// (void)gen_onoff_set_unack(mod_srv, &ctx, &msg);
		// return;
	//}
	printk("publish to 0x%04x onoff 0x%02x\n", pub_cli->addr, button->button_pressed);

	if (pub_cli->addr == BT_MESH_ADDR_UNASSIGNED) {
		return;
	}	
	
	bt_mesh_model_msg_init(pub_cli->msg, BT_MESH_MODEL_OP_GEN_ONOFF_SET);
	net_buf_simple_add_u8(pub_cli->msg, button->button_pressed); 
	net_buf_simple_add_u8(pub_cli->msg, trans_id++); //whithout this does not work
	err = bt_mesh_model_publish(mod_cli);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}
}

K_WORK_DEFINE(button0_work, button_pressed_worker);






/* Disable OOB security for SILabs Android app */

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
#if 1
	.output_size = 6,
	.output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
	.output_number = output_number,
	.output_string = output_string,
#else
	.output_size = 0,
	.output_actions = 0,
	.output_number = 0,
#endif
	.complete = prov_complete,
	.reset = prov_reset,
};

/*
 * Bluetooth Ready Callback
 */

static void bt_ready(int err)
{
	struct bt_le_oob oob;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* Use identity address as device UUID */
	if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob)) {
		printk("Identity Address unavailable\n");
	} else {
		memcpy(dev_uuid, oob.addr.a.val, 6);
	}

	bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);

	printk("Mesh initialized\n");
}



static void sw_pressed(const struct device *dev, struct gpio_callback *cb,
	uint32_t pin_pos) {
		
	// static uint32_t time, last_time;
	// time = k_uptime_get_32();

	// /* debounce the switch */
	// if (time < last_time + 250) { //250ms		
	// 	return;
	// }
	// last_time = time;

    struct gpio_dt_spec switches[] = {
		GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw1), okay)	
		GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
		GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
#endif
	};
	gpio_pin_t gpio_switches[] = {
		DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
#if DT_NODE_HAS_STATUS(DT_ALIAS(sw1), okay)	
		DT_GPIO_PIN(DT_ALIAS(sw1), gpios),
		DT_GPIO_PIN(DT_ALIAS(sw2), gpios),
#endif
	};
	for (size_t i = 0; i < ARRAY_SIZE(switches); i++)
	{		
		if (pin_pos == BIT(gpio_switches[i])){
			int state = gpio_pin_get(dev, gpio_switches[i]);
			printk("button %d changed, value is %d \n", i, state);
			//delegate to seperate worker to avoid blocking
			struct buttonwork button = {
        		.button_pressed = state,
				.button_num = i,
				.work = button0_work,
    		};
	
			k_work_submit(&button.work);
		}		
	}	
}

static struct gpio_callback button_cb;
//const static struct gpio_dt_spec sw_device0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});

static void callbackbuttons() {	
	struct gpio_dt_spec sw_device0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

#if DT_NODE_HAS_STATUS(DT_ALIAS(sw1), okay)	

	struct gpio_dt_spec sw_device1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
	struct gpio_dt_spec sw_device2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

	__ASSERT(device_is_ready(sw_device1.port), "Device not ready");
	__ASSERT(device_is_ready(sw_device2.port), "Device not ready");

	gpio_pin_configure_dt(&sw_device1, GPIO_INPUT);
	gpio_pin_configure_dt(&sw_device2, GPIO_INPUT);

	gpio_pin_interrupt_configure_dt(&sw_device1, GPIO_INT_EDGE_BOTH);
	gpio_pin_interrupt_configure_dt(&sw_device2, GPIO_INT_EDGE_BOTH);

	gpio_add_callback(sw_device1.port, &button_cb);
	gpio_add_callback(sw_device2.port, &button_cb);
	
	gpio_init_callback(&button_cb, sw_pressed, BIT(sw_device0.pin) | BIT(sw_device1.pin) | BIT(sw_device2.pin));
#else
	gpio_init_callback(&button_cb, sw_pressed, BIT(sw_device0.pin));
#endif
		
	__ASSERT(device_is_ready(sw_device0.port), "Device not ready");

	gpio_pin_configure_dt(&sw_device0, GPIO_INPUT);
	
	gpio_pin_interrupt_configure_dt(&sw_device0, GPIO_INT_EDGE_BOTH);
	
	gpio_add_callback(sw_device0.port, &button_cb);

}

// #ifdef CONFIG_USB_DEVICE_STACK
// void enable_usbcdc()
// {	
// 	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
// 	__ASSERT(device_is_ready(dev), "console not ready");
// 	uint32_t dtr = 0;

// 	if (usb_enable(NULL)) {
// 		return;
// 	}

// 	/* Poll if the DTR flag was set */
// 	while (!dtr) {
// 		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
// 		/* Give CPU resources to low priority threads. */
// 		k_sleep(K_MSEC(100));
// 	}	
// }
// #endif

int main(void)
{
	int err;
// #ifdef CONFIG_USB_DEVICE_STACK
// 	enable_usbcdc();
// 	console_init();
// 	k_sleep(K_MSEC(2000));
// 	printk("delete flash? y/n\n");
// 	uint8_t c = console_getchar();
// 	if (c == 'y') {
// 		printk("deleting flash\n");
// 	 	bt_mesh_reset();

// 	} else {
// 	 	printk("not deleting flash: %c\n", c);
// 	}
// #endif
	err = gpio_pin_configure_dt(&flashlight, GPIO_OUTPUT_INACTIVE);

	printk("Initializing...\n");
			
	callbackbuttons();
	err = init_clock();
	if(err) {
		printk("Error initializing shift register\n");
		return 0;
	}

	//k_work_init_delayable(&attention_blink_work, attention_blink);
	
	
	// Initialize LED's 
	
	for(int x=0; x< ARRAY_SIZE(onoff_state); x++) {
		__ASSERT(device_is_ready(onoff_state[x].led_device.port), "LED GPIO controller device is not ready\n");			
		gpio_pin_configure_dt(&onoff_state[x].led_device, GPIO_OUTPUT_INACTIVE);
	}	
	
	/* Initialize the Bluetooth Subsystem */	
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
			
	return 0;
}


