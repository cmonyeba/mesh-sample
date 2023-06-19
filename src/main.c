/*
 * HALFWORD - Mesh Example
 */

#include <zephyr/sys/printk.h>

#include <zephyr/settings/settings.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>

// Defines for node composition data
#define BT_COMP_ID_HALFWORD 0xBEEF
#define BT_PROD_ID_HALFWORD 0xABCD
#define BT_VER_ID_HALFWORD  0x0001

// Message opcodes
#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static void attention_on(struct bt_mesh_model *mod);
static void attention_off(struct bt_mesh_model *mod);

static int gen_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
static int gen_onoff_set(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
static int gen_onoff_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf);
static int onoff_status_send(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx);
static void onoff_timeout(struct k_work *work);

static int output_number(bt_mesh_output_action_t action, uint32_t number);
static void prov_complete(uint16_t net_idx, uint16_t addr);
static void prov_reset(void);
static void prov_reset(void);

static void bt_ready(int err);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static uint8_t dev_uuid[16];

// Element State
static struct {
	bool val;
	uint8_t tid;
	uint16_t src;
	uint32_t transition_time;
	struct k_work_delayable work;
} onoff;

// Health Server Model
static const struct bt_mesh_health_srv_cb health_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

// Message opcode handler array
static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ OP_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ OP_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
	{ OP_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

// Model array
static struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL,
		      NULL),
};

// Element array
static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

// Node composition
static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_HALFWORD,
	.pid = BT_PROD_ID_HALFWORD,
	.vid = BT_VER_ID_HALFWORD,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

// Node capabilities
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 1,
	.output_actions = BT_MESH_DISPLAY_NUMBER | BT_MESH_BLINK,
	.output_number = output_number,
	.complete = prov_complete,
	.reset = prov_reset,
};

// Time resolution field
static const uint32_t time_res[] = {
	100,
	MSEC_PER_SEC,
	10 * MSEC_PER_SEC,
	10 * 60 * MSEC_PER_SEC,
};

/*
 * Function: attention_on
 * -------------------------------------------------------------
 * description: sets attention state on
 * 
 * mod: reference to model instance
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static void attention_on(struct bt_mesh_model *mod)
{
	gpio_pin_set_dt(&led0, true);
}

/*
 * Function: attention_off
 * -------------------------------------------------------------
 * description: sets attention state off
 * 
 * mod: reference to model instance
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static void attention_off(struct bt_mesh_model *mod)
{
	gpio_pin_set_dt(&led0, false);
}

/*
 * Function: model_time_decode
 * -------------------------------------------------------------
 * description: decodes transition time
 * 
 * val: buffer value
 * 
 * returns: transition time
 * -------------------------------------------------------------
 */
static inline int32_t model_time_decode(uint8_t val)
{
	uint8_t resolution = (val >> 6) & BIT_MASK(2);
	uint8_t steps = val & BIT_MASK(6);

	if (steps == 0x3f) {
		return SYS_FOREVER_MS;
	}

	return steps * time_res[resolution];
}

/*
 * Function: model_time_encode
 * -------------------------------------------------------------
 * description: encodes transition time
 * 
 * ms: value in milliseconds.
 * 
 * returns: buffer value
 * -------------------------------------------------------------
 */
static inline uint8_t model_time_encode(int32_t ms)
{
	if (ms == SYS_FOREVER_MS) {
		return 0x3f;
	}

	for (int i = 0; i < ARRAY_SIZE(time_res); i++) {
		if (ms >= BIT_MASK(6) * time_res[i]) {
			continue;
		}

		uint8_t steps = DIV_ROUND_UP(ms, time_res[i]);

		return steps | (i << 6);
	}

	return 0x3f;
}

/*
 * Function: onoff_status_send 
 * -------------------------------------------------------------
 * description: Sends current state status of the model
 * 
 * model: reference to the mesh mode
 * ctx: message sending context
 * 
 * returns: 0 on success, or (negative) error code on failure.
 * -------------------------------------------------------------
 */
static int onoff_status_send(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx)
{
	uint32_t remaining;

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 3);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);
    
	// Determine how much time remains
	remaining = k_ticks_to_ms_floor32(
			    k_work_delayable_remaining_get(&onoff.work)) +
		    onoff.transition_time;

	/* 
	 * Check using remaining time instead of "work pending" to make the
	 * onoff status send the right value on instant transitions. As the
	 * work item is executed in a lower priority than the mesh message
	 * handler, the work will be pending even on instant transitions.
	 */
	if (remaining) {
		// Add time remaining to buffer
		net_buf_simple_add_u8(&buf, !onoff.val);
		net_buf_simple_add_u8(&buf, onoff.val);
		net_buf_simple_add_u8(&buf, model_time_encode(remaining));
	} else {
		// Add model value to buffer.
		net_buf_simple_add_u8(&buf, onoff.val);
	}

	return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
}

/*
 * Function: onoff_timeout
 * -------------------------------------------------------------
 * description: work item handler. the reason for using a work
 *              thread is to account for delay and transition
 * 
 * work: work object reference
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static void onoff_timeout(struct k_work *work)
{
	
	if (onoff.transition_time) 
	{	
		/* 
		 * LED should be on whenever there is a transistion in progress
		 * according to the Bluetooth Mesh Model specification, section 3.1.1.
	    */
		gpio_pin_set_dt(&led1, true);

		// Reschedule work object after the transition time expires
		k_work_reschedule(&onoff.work, K_MSEC(onoff.transition_time));
		onoff.transition_time = 0;
		return;
	}

	/* 
	* The boolean state occurs once the transition is done
	* or if there is no transition time
	*/
	gpio_pin_set_dt(&led1, onoff.val);
}

/*
 * Function: gen_onoff_get
 * -------------------------------------------------------------
 * description: sends the current state status of the model
 * 
 * work: work object reference
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static int gen_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	onoff_status_send(model, ctx);
	return 0;
}

/*
 * Function: gen_onoff_set_unack
 * -------------------------------------------------------------
 * description: work item handler. the reason for using a work
 *              thread is to account for delay and transition
 * 
 * work: work object reference
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static int gen_onoff_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{

	uint8_t val = net_buf_simple_pull_u8(buf);
	uint8_t tid = net_buf_simple_pull_u8(buf);
	int32_t trans = 0;
	int32_t delay = 0;

	// Get the transition and delay time
	if (buf->len) {
		trans = model_time_decode(net_buf_simple_pull_u8(buf));
		delay = net_buf_simple_pull_u8(buf) * 5;
	}

	// Only perform change if the message wasn't a duplicate  
	if (tid == onoff.tid && ctx->addr == onoff.src) {
		return 0;
	}


	// Only perform change if the message has different value 
	if (val == onoff.val) {
		return 0;
	}

	// Update Model State
	onoff.tid = tid;
	onoff.src = ctx->addr;
	onoff.val = val;
	onoff.transition_time = trans;

	/*
	 * Schedule the next action to happen on the delay, and keep
	 * transition time stored, so it can be applied in the timeout.
	 */ 
	k_work_reschedule(&onoff.work, K_MSEC(delay));

	return 0;
}

/*
 * Function: gen_onoff_set_unack
 * -------------------------------------------------------------
 * description: same as gen_onoff_set_unack except sends a status
 *              message to acknowledege
 * 
 * model: reference to mode instance
 * ctx: message sending context
 * buf: network buffer
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static int gen_onoff_set(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	(void)gen_onoff_set_unack(model, ctx, buf);
	onoff_status_send(model, ctx);

	return 0;
}

/*
 * Function: output_number
 * -------------------------------------------------------------
 * description: OOB output handler. outputs numeric value for
 *              authentication
 * 
 * number: OOB output value
 * action: OOB output action
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	switch(action)
	{
		case BT_MESH_NO_OUTPUT:
			break;
		case BT_MESH_BLINK:
		{
			uint32_t blink = number * 2;

			gpio_pin_set_dt(&led0, false);

			while(blink)
			{
				gpio_pin_toggle_dt(&led0);
				k_msleep(500);
				blink--;
			} 
		}
		case BT_MESH_BEEP:
		case BT_MESH_VIBRATE:
		case BT_MESH_DISPLAY_NUMBER:
			printk("OOB Number: %u\n", number);
			break;
		case BT_MESH_DISPLAY_STRING:
			break;
		default:
			break;
	}

	return 0;
}

/*
 * Function: prov_complete
 * -------------------------------------------------------------
 * description: callback when provisioning is completed
 * 
 * net_idx: network index
 * addr: network address
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	printk("Provisioning Complete");
}

/*
 * Function: prov_reset
 * -------------------------------------------------------------
 * description: callback when provisioning is completed
 * 
 * net_idx: network index
 * addr: network address
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

/*
 * Function: bt_ready
 * -------------------------------------------------------------
 * description: initalizes mesh support and enables specific 
 *              provisioing bearers to advertise and unprovisioned
 * 
 * err: error code
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
static void bt_ready(int err)
{
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

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

/*
 * Function: main
 * -------------------------------------------------------------
 * description: main routine where the program begins.
 * 
 * returns: n/a
 * -------------------------------------------------------------
 */
int main(void)
{
	int err = -1;

	printk("Initializing...\n");


	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}

	if (!gpio_is_ready_dt(&led0)) {
		return 0;
	}
	err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return 0;
	}
	if (!gpio_is_ready_dt(&led1)) {
		return 0;
	}
	err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return 0;
	}

	if (err) {
		printk("Board init failed (err: %d)\n", err);
		return 0;
	}

	k_work_init_delayable(&onoff.work, onoff_timeout);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	return 0;
}