/*
 *  PS3 System Manager.
 *
 *  Copyright (C) 2006 Sony Computer Entertainment Inc.
 *  Copyright 2006 Sony Corp.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define DEBUG 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <asm/ps3.h>
#include "vuart.h"

MODULE_AUTHOR("Sony Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PS3 System Manager");

/**
 * The system manager is an application running in the system policy module.
 * Guests communicate with the system manager through port 2 of the vuart using
 * a simple packaged(?) message protocol.  Messages are comprised of a fixed
 * field message header followed by a message specific payload.
 * One of the roles of the system manager is to coordinate proper system
 * reboot and shutdown sequences.
 *
 */

/**
 * struct ps3_sys_manager_header - System Manager message header.
 * @version: Header version, currently 1.
 * @size: Header size in bytes, curently 16.
 * @payload_size: Message payload size in bytes.
 * @service_id: Message type, one of enum ps3_sys_manager_service_id.
 */

struct ps3_sys_manager_header {
	/* version 1 */
	u8 version;
	u8 size;
	u16 reserved_1;
	u32 payload_size;
	u16 service_id;
	u16 reserved_2[3];
};

/**
 * @PS3_SM_RX_MSG_LEN - System manager received message length.
 *
 * Currently all messages received from the system manager are the same length
 * (16 bytes header + 16 bytes payload = 32 bytes).  This knowlege is used to
 * simplify the logic.
 */

enum {
	PS3_SM_RX_MSG_LEN = 32,
};

/**
 * enum ps3_sys_manager_service_id - Message header service_id.
 * @PS3_SM_SERVICE_ID_REQUEST:      guest --> sys_manager.
 * @PS3_SM_SERVICE_ID_COMMAND:      guest <-- sys_manager.
 * @PS3_SM_SERVICE_ID_RESPONSE:     guest --> sys_manager.
 * @PS3_SM_SERVICE_ID_SET_ATTR:     guest --> sys_manager.
 * @PS3_SM_SERVICE_ID_EXTERN_EVENT: guest <-- sys_manager.
 * @PS3_SM_SERVICE_ID_SET_NEXT_OP:  guest --> sys_manager.
 */

enum ps3_sys_manager_service_id {
	/* version 1 */
	PS3_SM_SERVICE_ID_REQUEST = 1,
	PS3_SM_SERVICE_ID_RESPONSE = 2,
	PS3_SM_SERVICE_ID_COMMAND = 3,
	PS3_SM_SERVICE_ID_EXTERN_EVENT = 4,
	PS3_SM_SERVICE_ID_SET_NEXT_OP = 5,
	PS3_SM_SERVICE_ID_SET_ATTR = 8,
};

/**
 * enum ps3_sys_manager_attr - Notification attribute (bit position mask).
 * @PS3_SM_ATTR_POWER: Power button.
 * @PS3_SM_ATTR_RESET: Reset button, not available on retail console.
 * @PS3_SM_ATTR_THERMAL: Sytem thermal alert.
 * @PS3_SM_ATTR_CONTROLLER: Remote controller event.
 * @PS3_SM_ATTR_ALL: Logical OR of all.
 *
 * The guest tells the system manager which events it is interested in receiving
 * notice of by sending the system manager a logical OR of notification
 * attributes via the ps3_sys_manager_send_attr() routine.
 */

enum ps3_sys_manager_attr {
	/* version 1 */
	PS3_SM_ATTR_POWER = 1,
	PS3_SM_ATTR_RESET = 2,
	PS3_SM_ATTR_THERMAL = 4,
	PS3_SM_ATTR_CONTROLLER = 8, /* bogus? */
	PS3_SM_ATTR_ALL = 0x0f,
};

/**
 * enum ps3_sys_manager_event - External event type, reported by system manager.
 * @PS3_SM_EVENT_POWER_PRESSED: payload.value not used.
 * PS3_SM_EVENT_POWER_RELEASED: payload.value = time pressed in millisec.
 * PS3_SM_EVENT_RESET_PRESSED: payload.value not used.
 * PS3_SM_EVENT_RESET_RELEASED: payload.value = time pressed in millisec.
 * PS3_SM_EVENT_THERMAL_ALERT: payload.value = thermal zone id.
 * PS3_SM_EVENT_THERMAL_CLEARED: payload.value = thermal zone id.
 */

enum ps3_sys_manager_event {
	/* version 1 */
	PS3_SM_EVENT_POWER_PRESSED = 3,
	PS3_SM_EVENT_POWER_RELEASED = 4,
	PS3_SM_EVENT_RESET_PRESSED = 5,
	PS3_SM_EVENT_RESET_RELEASED = 6,
	PS3_SM_EVENT_THERMAL_ALERT = 7,
	PS3_SM_EVENT_THERMAL_CLEARED = 8,
	/* no info on controller events */
};

/**
 * enum ps3_sys_manager_next_op - Operation to perform after lpar is destroyed.
 */

enum ps3_sys_manager_next_op {
	/* version 3 */
	PS3_SM_NEXT_OP_SYS_SHUTDOWN = 1,
	PS3_SM_NEXT_OP_SYS_REBOOT = 2,
	PS3_SM_NEXT_OP_LPAR_REBOOT = 0x82,
};

/**
 * enum ps3_sys_manager_wake_source - Next-op wakeup source (bit position mask).
 * @PS3_SM_WAKE_DEFAULT: Disk insert, power button, eject button, IR
 * controller, and bluetooth controller.
 * @PS3_SM_WAKE_RTC: .
 * @PS3_SM_WAKE_RTC_ERROR: .
 * @PS3_SM_WAKE_P_O_R: Power on reset.
 *
 * Additional wakeup sources when specifying PS3_SM_NEXT_OP_SYS_SHUTDOWN.
 * System will always wake from the PS3_SM_WAKE_DEFAULT sources.
 */

enum ps3_sys_manager_wake_source {
	/* version 3 */
	PS3_SM_WAKE_DEFAULT   = 0,
	PS3_SM_WAKE_RTC       = 0x00000040,
	PS3_SM_WAKE_RTC_ERROR = 0x00000080,
	PS3_SM_WAKE_P_O_R     = 0x10000000,
};

/**
 * enum ps3_sys_manager_cmd - Command from system manager to guest.
 *
 * The guest completes the actions needed, then acks or naks the command via
 * ps3_sys_manager_send_response().  In the case of @PS3_SM_CMD_SHUTDOWN,
 * the guest must be fully prepared for a system reset prior to acking the
 * command.
 */

enum ps3_sys_manager_cmd {
	/* version 1 */
	PS3_SM_CMD_SHUTDOWN = 1, /* shutdown guest OS */
};

/**
 * ps3_sys_manager_write - Helper to write a two part message to the vuart.
 *
 */

static int ps3_sys_manager_write(struct ps3_vuart_port_device *dev,
	const struct ps3_sys_manager_header *header, const void *payload)
{
	int result;

	BUG_ON(header->version != 1);
	BUG_ON(header->size != 16);
	BUG_ON(header->payload_size != 8 && header->payload_size != 16);
	BUG_ON(header->service_id > 8);

	result = ps3_vuart_write(dev, header,
		sizeof(struct ps3_sys_manager_header));
	
	if (!result)
		result = ps3_vuart_write(dev, payload, header->payload_size);
	
	return result;
}

/**
 * ps3_sys_manager_send_attr - Send a 'set attribute' to the system manager.
 *
 */

static int ps3_sys_manager_send_attr(struct ps3_vuart_port_device *dev,
	enum ps3_sys_manager_attr attr)
{
	static const struct ps3_sys_manager_header header = {
		.version = 1,
		.size = 16,
		.payload_size = 16,
		.service_id = PS3_SM_SERVICE_ID_SET_ATTR,
	};
	struct {
		u8 version;
		u8 reserved_1[3];
		u32 attribute;
	} payload;

	BUILD_BUG_ON(sizeof(payload) != 8);

	dev_dbg(&dev->core, "%s:%d: %xh\n", __func__, __LINE__, attr);

	memset(&payload, 0, sizeof(payload));
	payload.version = 1;
	payload.attribute = attr;

	return ps3_sys_manager_write(dev, &header, &payload);
}

/**
 * ps3_sys_manager_send_next_op - Send a 'set next op' to the system manager.
 *
 * Tell the system manager what to do after this lpar is destroyed.
 */

static int ps3_sys_manager_send_next_op(struct ps3_vuart_port_device *dev,
	enum ps3_sys_manager_next_op op,
	enum ps3_sys_manager_wake_source wake_source)
{
	static const struct ps3_sys_manager_header header = {
		.version = 1,
		.size = 16,
		.payload_size = 16,
		.service_id = PS3_SM_SERVICE_ID_SET_NEXT_OP,
	};
	struct {
		u8 version;
		u8 type;
		u8 gos_id;
		u8 reserved_1;
		u32 wake_source;
		u8 reserved_2[8];
	} payload;

	BUILD_BUG_ON(sizeof(payload) != 16);

	dev_dbg(&dev->core, "%s:%d: (%d)\n", __func__, __LINE__, op);

	memset(&payload, 0, sizeof(payload));
	payload.version = 3;
	payload.type = op;
	payload.gos_id = 3; /* other os */
	payload.wake_source = wake_source;

	return ps3_sys_manager_write(dev, &header, &payload);
}

/**
 * ps3_sys_manager_send_request_shutdown - Send 'request' to the system manager.
 *
 * The guest sends this message to request an operation or action.  The reply
 * is a command message from the system manager.  The guest performs the
 * requested opperation in the command handler.  The result of the command is
 * then communicated back to the system manager with a with a response message.
 * Currently the only supported request it the 'shutdown self' request.
 */

static int ps3_sys_manager_send_request_shutdown(struct ps3_vuart_port_device *dev)
{
	static const struct ps3_sys_manager_header header = {
		.version = 1,
		.size = 16,
		.payload_size = 16,
		.service_id = PS3_SM_SERVICE_ID_REQUEST,
	};
	struct {
		u8 version;
		u8 type;
		u8 gos_id;
		u8 reserved_1[13];
	} static const payload = {
		.version = 1,
		.type = 1, /* shutdown */
		.gos_id = 0, /* self */
	};

	BUILD_BUG_ON(sizeof(payload) != 16);

	dev_dbg(&dev->core, "%s:%d\n", __func__, __LINE__);

	return ps3_sys_manager_write(dev, &header, &payload);
}

/**
 * ps3_sys_manager_send_response - Send a 'response' to the system manager.
 * @status: zero = success, others fail.
 *
 * The guest sends this message to the system manager to acnowledge suscess or
 * failure of a command sent by the system manager.
 */

static int ps3_sys_manager_send_response(struct ps3_vuart_port_device *dev,
	u64 status)
{
	static const struct ps3_sys_manager_header header = {
		.version = 1,
		.size = 16,
		.payload_size = 16,
		.service_id = PS3_SM_SERVICE_ID_RESPONSE,
	};
	struct {
		u8 version;
		u8 reserved_1[3];
		u8 status;
		u8 reserved_2[11];
	} payload;

	BUILD_BUG_ON(sizeof(payload) != 16);

	dev_dbg(&dev->core, "%s:%d: (%s)\n", __func__, __LINE__,
		(status ? "nak" : "ack");

	memset(&payload, 0, sizeof(payload));
	payload.version = 1;
	payload.status = status;

	return ps3_sys_manager_write(dev, &header, &payload);
}

/**
 * ps3_sys_manager_handle_event - Second stage asyncronous event handler.
 *
 */

static void ps3_sys_manager_handle_event(struct ps3_vuart_port_device *dev)
{
	int result;
	struct {
		u8 version;
		u8 type;
		u8 reserved_1[2];
		u32 value;
		u8 reserved_2[8];
	} event;

	BUILD_BUG_ON(sizeof(event) != 16);

	dev_dbg(&dev->core, "%s:%d: -> \n", __func__, __LINE__);

	result = ps3_vuart_read(dev, &event, sizeof(event));
	BUG_ON(result);

	if (event.version != 1) {
		dev_dbg(&dev->core, "%s:%d: unsupported event version (%u)\n",
			__func__, __LINE__, event.version);
		return;
	}

	switch (event.type) {
	case PS3_SM_EVENT_POWER_PRESSED:
		dev_dbg(&dev->core, "%s:%d: POWER_PRESSED\n",
			__func__, __LINE__);
		break;
	case PS3_SM_EVENT_POWER_RELEASED:
		dev_dbg(&dev->core, "%s:%d: POWER_RELEASED (%u ms)\n",
			__func__, __LINE__, event.value);
		ps3_sys_manager_send_next_op(dev, PS3_SM_NEXT_OP_SYS_SHUTDOWN,
			PS3_SM_WAKE_DEFAULT);
		ps3_sys_manager_send_request_shutdown(dev);
		break;
	case PS3_SM_EVENT_THERMAL_ALERT:
		dev_dbg(&dev->core, "%s:%d: THERMAL_ALERT (zone %u)\n",
			__func__, __LINE__, event.value);
		break;
	case PS3_SM_EVENT_THERMAL_CLEARED:
		dev_dbg(&dev->core, "%s:%d: THERMAL_CLEARED (zone %u)\n",
			__func__, __LINE__, event.value);
		break;
	default:
		dev_dbg(&dev->core, "%s:%d: unknown event (%u)\n",
			__func__, __LINE__, event.type);
		break;
	}

	dev_dbg(&dev->core, "%s:%d: <- \n", __func__, __LINE__);
}
/**
 * ps3_sys_manager_handle_cmd - Second stage asyncronous command handler.
 *
 * The system manager sends this in reply to a 'request' message from the guest.
 */

static void ps3_sys_manager_handle_cmd(struct ps3_vuart_port_device *dev)
{
	int result;
	struct {
		u8 version;
		u8 type;
		u8 reserved_1[14];
	} cmd;

	BUILD_BUG_ON(sizeof(cmd) != 16);

	dev_dbg(&dev->core, "%s:%d: -> \n", __func__, __LINE__);

	result = ps3_vuart_read(dev, &cmd, sizeof(cmd));
	BUG_ON(result);

	if (cmd.version != 1) {
		dev_dbg(&dev->core, "%s:%d: unsupported cmd version (%u)\n",
			__func__, __LINE__, cmd.version);
		return;
	}

	if (cmd.type != PS3_SM_CMD_SHUTDOWN) {
		dev_dbg(&dev->core, "%s:%d: unknown cmd (%u)\n",
			__func__, __LINE__, cmd.type);
		return;
	}

	//ctrl_alt_del();

	//need to call this with success somehow when the kernel is ready...
	ps3_sys_manager_send_response(dev, 0);

	dev_dbg(&dev->core, "%s:%d: <- \n", __func__, __LINE__);
}

/**
 * ps3_sys_manager_handle_msg - First stage asyncronous msg handler.
 *
 * Signaled when a complete message arrives at the vuart port.
 */

static void ps3_sys_manager_handle_msg(struct work_struct *work)
{
	int result;
	struct ps3_vuart_port_device *dev = ps3_vuart_work_to_port_device(work);
	struct ps3_sys_manager_header header;

	dev_dbg(&dev->core, "%s:%d: -> \n", __func__, __LINE__);

	result = ps3_vuart_read(dev, &header,
		sizeof(struct ps3_sys_manager_header));

	BUG_ON(result);

	if (header.version != 1) {
		dev_dbg(&dev->core, "%s:%d: unsupported header version (%u)\n",
			__func__, __LINE__, header.version);
		goto fail_header;
	}

	BUILD_BUG_ON(sizeof(header) != 16);
	BUG_ON(header.size != 16);
	BUG_ON(header.payload_size != 16);

	switch (header.service_id) {
	case PS3_SM_SERVICE_ID_EXTERN_EVENT:
		dev_dbg(&dev->core, "%s:%d: EVENT\n", __func__, __LINE__);
		ps3_sys_manager_handle_event(dev);
		break;
	case PS3_SM_SERVICE_ID_COMMAND:
		dev_dbg(&dev->core, "%s:%d: COMMAND\n", __func__, __LINE__);
		ps3_sys_manager_handle_cmd(dev);
		break;
	default:
		dev_dbg(&dev->core, "%s:%d: unknown service_id (%u)\n",
			__func__, __LINE__, header.service_id);
		goto fail_id;
	}
	goto read_next;

fail_header:
	// clear rx buffer here!!!
	BUG();
	goto read_next;

fail_id:
	// read payload bytes here!!!
	BUG();
	goto read_next;

read_next:
	result = ps3_vuart_read_async(dev, ps3_sys_manager_handle_msg,
		PS3_SM_RX_MSG_LEN);
	BUG_ON(result);
	dev_dbg(&dev->core, "%s:%d: <- \n", __func__, __LINE__);
}

static int ps3_sys_manager_probe(struct ps3_vuart_port_device *dev)
{
	int result;
	dev_dbg(&dev->core, "%s:%d: -> \n", __func__, __LINE__);

	result = ps3_sys_manager_send_attr(dev, PS3_SM_ATTR_ALL);
	BUG_ON(result);

	result = ps3_vuart_read_async(dev, ps3_sys_manager_handle_msg,
		PS3_SM_RX_MSG_LEN);
	BUG_ON(result);

	dev_dbg(&dev->core, "%s:%d: <- \n", __func__, __LINE__);
	return result;
}

static int
ps3_sys_manager_remove (struct ps3_vuart_port_device *dev)
{
	dev_dbg(&dev->core, "%s:%d: -> \n", __func__, __LINE__);
	BUG_ON("do");
	dev_dbg(&dev->core, "%s:%d: <- \n", __func__, __LINE__);
	return 0;
}

static struct ps3_vuart_port_driver ps3_sys_manager = {
	.match_id = PS3_MATCH_ID_SYSTEM_MANAGER,
	.core = {
		.name = "ps3_sys_manager",
	},
	.probe = ps3_sys_manager_probe,
	.remove = ps3_sys_manager_remove,
};

static int __init
ps3_sys_manager_init (void)
{
	int result;

	pr_debug(" -> %s:%d\n", __func__, __LINE__);
	result = ps3_vuart_port_driver_register(&ps3_sys_manager);
	pr_debug(" <- %s:%d\n", __func__, __LINE__);
	return result;
}

static void __exit
ps3_sys_manager_exit (void)
{
	pr_debug(" -> %s:%d\n", __func__, __LINE__);
	ps3_vuart_port_driver_unregister(&ps3_sys_manager);
	pr_debug(" <- %s:%d\n", __func__, __LINE__);
}

module_init (ps3_sys_manager_init);
module_exit (ps3_sys_manager_exit);

