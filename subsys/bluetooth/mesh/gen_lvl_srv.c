/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
 
#include <string.h>
#include <bluetooth/mesh/gen_lvl_srv.h>
#include <bluetooth/mesh/gen_dtt_srv.h>
#include <bluetooth/mesh/scene_srv.h>
#include <sys/byteorder.h>
#include "model_utils.h"
#define CHAMP 
#ifdef CHAMP
extern int setDeviceTime(uint32_t newDeviceTime);
extern void setPhotoControl(bool daylightHarvestingFlag);
extern void setOccupancyDelayTime(uint16_t occupancyDelayTimeout);
extern void setPhotoControlRefLuminance(uint16_t photoControlRefLuminance);
extern void setActiveControlprofileId(uint8_t activeControlprofileId);
extern void setDayLightSavingsDates(uint32_t dayLightSavingStartDate,uint32_t dayLightSavingEndDate);
extern void setBlinkOnIdentifyAfterProvisioning(bool blink);
static void dummyResponse(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx);
//TODO check which all variables are really needed. and add comments
uint8_t currentDay =0;
//TODO rename it to some correct name other than debug
uint8_t debugMinMaxPerSlot[7][8] ={0};
uint16_t debugTimeSlot[7][4] ={0};
uint8_t tid = 0;
bool photoControlFlag = false;
uint16_t occupancyDelayTime = 0;
uint16_t referenceLuminanceLvl = 0;
uint8_t activeCtrlProfileID = 0;
uint8_t luminaireVariant = 0;
uint32_t daylightSavingStartDayEpochCount = 0;
uint32_t daylightSavingEndDayEpochCount = 0;
bool fixtureBlinkPostProvisioning = 0;
#endif

static void encode_status(const struct bt_mesh_lvl_status *status,
			  struct net_buf_simple *buf)
{
	bt_mesh_model_msg_init(buf, BT_MESH_LVL_OP_STATUS);
	net_buf_simple_add_le16(buf, status->current);

	if (status->remaining_time == 0) {
		return;
	}

	net_buf_simple_add_le16(buf, status->target);
	net_buf_simple_add_u8(buf,
			      model_transition_encode(status->remaining_time));
}

#ifdef CHAMP
static void encode_status_champ(const struct bt_mesh_lvl_champ_status *status,
			  struct net_buf_simple *buf)
{
	bt_mesh_model_msg_init(buf, status->opcode);
	net_buf_simple_add_u8(buf, status->data[0]);
    net_buf_simple_add_u8(buf, status->data[1]);
}
#endif
static void rsp_status(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		       const struct bt_mesh_lvl_status *status)
{
	BT_MESH_MODEL_BUF_DEFINE(rsp, BT_MESH_LVL_OP_STATUS,
				 BT_MESH_LVL_MSG_MAXLEN_STATUS);
	encode_status(status, &rsp);

	bt_mesh_model_send(model, ctx, &rsp, NULL, 0);
}

#ifdef CHAMP
static void rsp_champ_status(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		       const struct bt_mesh_lvl_champ_status *status)
{
	BT_MESH_MODEL_BUF_DEFINE(rsp, status->opcode,
				 2);
	encode_status_champ(status, &rsp);
	bt_mesh_model_send(model, ctx, &rsp, NULL, 0);
}
#endif
static int handle_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		      struct net_buf_simple *buf)
{
	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_lvl_status status = { 0 };

	srv->handlers->get(srv, ctx, &status);

	rsp_status(model, ctx, &status);

	return 0;
}
uint8_t debugGenLvl=0;
static int set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		struct net_buf_simple *buf, bool ack)
{
	if (buf->len != BT_MESH_LVL_MSG_MINLEN_SET &&
	    buf->len != BT_MESH_LVL_MSG_MAXLEN_SET) {
		return -EMSGSIZE;
	}

	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_model_transition transition;
	struct bt_mesh_lvl_status status = { 0 };
	struct bt_mesh_lvl_set set;

	set.lvl = net_buf_simple_pull_le16(buf);
	set.new_transaction = !tid_check_and_update(
		&srv->tid, net_buf_simple_pull_u8(buf), ctx);
	set.transition = model_transition_get(srv->model, &transition, buf);
        debugGenLvl++;
	srv->handlers->set(srv, ctx, &set, &status);

	if (IS_ENABLED(CONFIG_BT_MESH_SCENE_SRV)) {
		bt_mesh_scene_invalidate(srv->model);
	}

	if (ack) {
		rsp_status(model, ctx, &status);
	}

	(void)bt_mesh_lvl_srv_pub(srv, NULL, &status);

	return 0;
}

static int delta_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		      struct net_buf_simple *buf, bool ack)
{
	if (buf->len != BT_MESH_LVL_MSG_MINLEN_DELTA_SET &&
	    buf->len != BT_MESH_LVL_MSG_MAXLEN_DELTA_SET) {
		return -EMSGSIZE;
	}

	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_lvl_status status = { 0 };
	struct bt_mesh_lvl_delta_set delta_set;
	struct bt_mesh_model_transition transition;

	delta_set.delta = net_buf_simple_pull_le32(buf);
	delta_set.new_transaction = !tid_check_and_update(
		&srv->tid, net_buf_simple_pull_u8(buf), ctx);
	delta_set.transition = model_transition_get(srv->model, &transition, buf);

	srv->handlers->delta_set(srv, ctx, &delta_set, &status);

	if (IS_ENABLED(CONFIG_BT_MESH_SCENE_SRV)) {
		bt_mesh_scene_invalidate(srv->model);
	}

	if (ack) {
		rsp_status(model, ctx, &status);
	}

	(void)bt_mesh_lvl_srv_pub(srv, NULL, &status);

	return 0;
}

static int move_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		     struct net_buf_simple *buf, bool ack)
{
	if (buf->len != BT_MESH_LVL_MSG_MINLEN_MOVE_SET &&
	    buf->len != BT_MESH_LVL_MSG_MAXLEN_MOVE_SET) {
		return -EMSGSIZE;
	}

	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_lvl_status status = { 0 };
	struct bt_mesh_model_transition transition;
	struct bt_mesh_lvl_move_set move_set;

	move_set.delta = net_buf_simple_pull_le16(buf);
	move_set.new_transaction = !tid_check_and_update(
		&srv->tid, net_buf_simple_pull_u8(buf), ctx);
	move_set.transition = model_transition_get(srv->model, &transition, buf);

	/* If transition.time is 0, we shouldn't move. Align these two
	 * parameters to simplify application logic for this case:
	 */
	if ((!move_set.transition || move_set.transition->time == 0) ||
	    move_set.delta == 0) {
		move_set.delta = 0;
		move_set.transition = NULL;
	}

	srv->handlers->move_set(srv, ctx, &move_set, &status);

	if (IS_ENABLED(CONFIG_BT_MESH_SCENE_SRV)) {
		bt_mesh_scene_invalidate(srv->model);
	}

	if (ack) {
		rsp_status(model, ctx, &status);
	}

	(void)bt_mesh_lvl_srv_pub(srv, ctx, &status);

	return 0;
}

/* Message handlers */

static int handle_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		      struct net_buf_simple *buf)
{
	return set(model, ctx, buf, true);
}

static int handle_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{
	return set(model, ctx, buf, false);
}

static int handle_delta_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{
	return delta_set(model, ctx, buf, true);
}

static int handle_delta_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
	return delta_set(model, ctx, buf, false);
}

static int handle_move_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			   struct net_buf_simple *buf)
{
	return move_set(model, ctx, buf, true);
}

static int handle_move_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				 struct net_buf_simple *buf)
{
	return move_set(model, ctx, buf, false);
}

#ifdef CHAMP
uint64_t currentTime =0;
static void handle_set_time(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
      //uint16_t pading = net_buf_simple_pull_le16(buf);  
      //uint8_t tid = net_buf_simple_pull_u8(buf); 
      currentTime = net_buf_simple_pull_le64(buf); 
      struct bt_mesh_lvl_champ_status status = { 0 };
      //call function from upper layer to pass the received epoch time
      setDeviceTime((uint32_t )currentTime);
      //TODO correct this response in wave 1
       rsp_status(model, ctx, &status);
}
static void handle_set_ScheduleData(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
        int index = 0;
        for(index=0;index<7;index++)//all 7 days
        {          
          //currentDay = net_buf_simple_pull_u8(buf);                       
          //min of slot 0          
          debugMinMaxPerSlot[index][0] = net_buf_simple_pull_u8(buf);  
          //if(index==0)
          //{
          //  tid = net_buf_simple_pull_u8(buf);
          //}
          //max of slot 0
          debugMinMaxPerSlot[index][1] = net_buf_simple_pull_u8(buf); 
          //slot 0 end //start is always 0:00 for slot 0
           
          //min slot 2
          debugMinMaxPerSlot[index][2] = net_buf_simple_pull_u8(buf);
          //max slot 2
          debugMinMaxPerSlot[index][3] = net_buf_simple_pull_u8(buf); 
          //end slot 2 . start is always slot 0 end +1 min

          //slot 3 min
          debugMinMaxPerSlot[index][4] = net_buf_simple_pull_u8(buf);  
          //slot 3 max
          debugMinMaxPerSlot[index][5] = net_buf_simple_pull_u8(buf); 
          //slot 3 end. start of lsot 3 is slot 2 end +1 min
          //slot 4 min
          debugMinMaxPerSlot[index][6] = net_buf_simple_pull_u8(buf); 
           //slot 4 max
          debugMinMaxPerSlot[index][7] = net_buf_simple_pull_u8(buf);
          //slot 4 end will always be 23:59. so no need of the actual value to be received

        }
        debugTimeSlot[0][0] = net_buf_simple_pull_le16(buf);
        debugTimeSlot[0][1] = net_buf_simple_pull_le16(buf); 
        debugTimeSlot[0][2] = net_buf_simple_pull_le16(buf); 
        debugTimeSlot[0][3] = 2359;//net_buf_simple_pull_le16(buf); 

        //struct bt_mesh_lvl_champ_status status = { 0 };
        //uint8_t ret = true;
        //status.data[0] = ret;
        //status.length = 1;
        //status.opcode = BT_MESH_LVL_STATUS_CHAMP_PROPRITERY;
        //rsp_status(model, ctx, &status);
        dummyResponse(model, ctx);
}
static void handle_set_occupancy_delay_time(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
  uint16_t occupancyDelayTimeout = net_buf_simple_pull_le16(buf); 
  setOccupancyDelayTime(occupancyDelayTimeout);
  struct bt_mesh_lvl_status status = { 0 };
  //rsp_status(model, ctx, &status);
  dummyResponse(model, ctx);

}
static void handle_set_photocontrol(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
   uint8_t daylightHarvestingFlag = net_buf_simple_pull_u8(buf);
   setPhotoControl((bool)daylightHarvestingFlag);
   //struct bt_mesh_lvl_status status = { 0 };
  //rsp_status(model, ctx, &status);
   dummyResponse(model, ctx);

}
static void handle_set_photocontrol_ref_luminance(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
  uint8_t photoControlRefLuminanceLSB = net_buf_simple_pull_u8(buf);
  uint8_t photoControlRefLuminanceMSB = net_buf_simple_pull_u8(buf);

  setPhotoControlRefLuminance((uint16_t)(photoControlRefLuminanceMSB<<8 | photoControlRefLuminanceLSB) );
//struct bt_mesh_lvl_status status = { 0 };
//  rsp_status(model, ctx, &status);
 dummyResponse(model, ctx);
}
static void handle_set_active_cp_id(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
  uint8_t activeControlprofileId = net_buf_simple_pull_u8(buf);
  setActiveControlprofileId(activeControlprofileId);
//struct bt_mesh_lvl_status status = { 0 };
//  rsp_status(model, ctx, &status);
 dummyResponse(model, ctx);
}
//uint8_t grf;
extern void store_champ_Lumen(uint8_t val);
static void handle_set_fixture_lumen_value(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
   uint8_t fixtureLumenValue = net_buf_simple_pull_u8(buf);
  // store_champ_Lumen(fixtureLumenValue);
  // struct bt_mesh_lvl_status status = { 0 };
  //rsp_status(model, ctx, &status);
   //grf = fixtureLumenValue;
   //TODO Store it in Nv memory  
    dummyResponse(model, ctx);
}
static void handle_set_daylight_saving_dates(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
  uint16_t dayLightSavingStartDateMSB = net_buf_simple_pull_le16(buf);
  uint16_t dayLightSavingStartDateLSB = net_buf_simple_pull_le16(buf); 
  uint16_t dayLightSavingEndDateMSB = net_buf_simple_pull_le16(buf);
  uint16_t dayLightSavingEndDateLSB = net_buf_simple_pull_le16(buf);
  uint32_t dayLightSavingStartDate = (uint32_t)(dayLightSavingStartDateMSB << 16 | dayLightSavingStartDateLSB) ;
  uint32_t dayLightSavingEndDate = (uint32_t)(dayLightSavingEndDateMSB << 16 | dayLightSavingEndDateLSB) ;
  setDayLightSavingsDates(dayLightSavingStartDate,dayLightSavingEndDate);
  //struct bt_mesh_lvl_status status = { 0 };
  //rsp_status(model, ctx, &status);
  dummyResponse(model, ctx);
}

static void handle_fixture_blink_after_provisioning(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
   uint8_t blink = net_buf_simple_pull_u8(buf);
   setBlinkOnIdentifyAfterProvisioning((bool)blink);
  // struct bt_mesh_lvl_status status = { 0 };
  //rsp_status(model, ctx, &status);
   dummyResponse(model, ctx);
}

extern void getFwVersion(uint8_t *pVersion);
static void handle_fixture_fw_version(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx,
				  struct net_buf_simple *buf)
{
    uint8_t fwVersion[2] ={0}; 
    getFwVersion(&fwVersion);

    struct bt_mesh_lvl_champ_status status = { 0 };
       status.data[0] = fwVersion[0];
       status.data[1] = fwVersion[1];
    status.length = 8;
    status.opcode = BT_MESH_FIXTURE_FW_VERSION;
    rsp_champ_status(model, ctx, &(status.data[0]));

}

static void dummyResponse(struct bt_mesh_model *model,
				  struct bt_mesh_msg_ctx *ctx)
{
   uint8_t fwVersion[2];
    getFwVersion(&fwVersion);

    struct bt_mesh_lvl_champ_status status = { 0 };
    status.data[0] = fwVersion[0];
    status.data[1] = fwVersion[1];
    status.length = 8;
    status.opcode = BT_MESH_FIXTURE_FW_VERSION;
    rsp_champ_status(model, ctx, &(status.data[0]));
}
#endif
const struct bt_mesh_model_op _bt_mesh_lvl_srv_op[] = {
	{
		BT_MESH_LVL_OP_GET,
		BT_MESH_LEN_EXACT(BT_MESH_LVL_MSG_LEN_GET),
		handle_get,
	},
	{
		BT_MESH_LVL_OP_SET,
		BT_MESH_LEN_MIN(BT_MESH_LVL_MSG_MINLEN_SET),
		handle_set,
	},
	{
		BT_MESH_LVL_OP_SET_UNACK,
		BT_MESH_LEN_MIN(BT_MESH_LVL_MSG_MINLEN_SET),
		handle_set_unack,
	},
	{
		BT_MESH_LVL_OP_DELTA_SET,
		BT_MESH_LEN_MIN(BT_MESH_LVL_MSG_MINLEN_DELTA_SET),
		handle_delta_set,
	},
	{
		BT_MESH_LVL_OP_DELTA_SET_UNACK,
		BT_MESH_LEN_MIN(BT_MESH_LVL_MSG_MINLEN_DELTA_SET),
		handle_delta_set_unack,
	},
	{
		BT_MESH_LVL_OP_MOVE_SET,
		BT_MESH_LEN_MIN(BT_MESH_LVL_MSG_MINLEN_MOVE_SET),
		handle_move_set,
	},
	{
		BT_MESH_LVL_OP_MOVE_SET_UNACK,
		BT_MESH_LEN_MIN(BT_MESH_LVL_MSG_MINLEN_MOVE_SET),
		handle_move_set_unack,
	},
#ifdef CHAMP	
	{ BT_MESH_LVL_SET_CHAMP_TIME, BT_MESH_LVL_MSG_MINLEN_SET_TIME,
	  handle_set_time },
        { BT_MESH_LVL_SET_CHAMP_SCHEDULEDATA, BT_MESH_LVL_MSG_MINLEN_SET_SCHEDULE,
	  handle_set_ScheduleData },
        { BT_MESH_LVL_SET_CHAMP_DAY_LIGHT_HARVESTIN_MODE, BT_MESH_PHOTO_CONTROL_MSG_LEN,
	  handle_set_photocontrol },
        { BT_MESH_LVL_SET_CHAMP_OCCUPANCY_DELAY_TIME, BT_MESH_OCCUPANCY_DELAY_TIME_MSG_LEN,
	  handle_set_occupancy_delay_time },
        { BT_MESH_LVL_SET_CHAMP_LIGHT_REFERENCE_LEVEL, BT_MESH_PHOTO_CONTROL_REFERENCE_LUMINANCE_MSG_LEN,
	  handle_set_photocontrol_ref_luminance },
          { BT_MESH_LVL_SET_CHAMP_ACTIVE_CONTROL_PROFILE, BT_MESH_ACTIVE_CONTROL_PROFILE_MSG_LEN,
	  handle_set_active_cp_id },
        { BT_MESH_LVL_SET_CHAMP_LUMEN_MAX, BT_MESH_DEVICE_MAX_LUMEN_LVL_MSG_LEN,
	  handle_set_fixture_lumen_value },
        { BT_MESH_LVL_SET_CHAMP_DAY_LIGHT_SAVING_DATES, BT_MESH_DAYLIGHT_SAVING_DATES_EPOCH_MSG_LEN,
	  handle_set_daylight_saving_dates },
          { BT_MESH_FIXTURE_BLINK_AFTER_PROVISIONING, BT_MESH_FIXTURE_BLINK_AFTER_PROVISIONING_MSG_LEN,
	  handle_fixture_blink_after_provisioning },
        { BT_MESH_FIXTURE_FW_VERSION, BT_MESH_FIXTURE_FW_VERSION_LEN,
	  handle_fixture_fw_version },
#endif
	BT_MESH_MODEL_OP_END,
};

static ssize_t scene_store(struct bt_mesh_model *model, uint8_t data[])
{
	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_lvl_status status = { 0 };

	srv->handlers->get(srv, NULL, &status);
	sys_put_le16(status.remaining_time ? status.target : status.current,
		     &data[0]);

	return 2;
}

static void scene_recall(struct bt_mesh_model *model, const uint8_t data[],
			 size_t len,
			 struct bt_mesh_model_transition *transition)
{
	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_lvl_status status = { 0 };
	struct bt_mesh_lvl_set set = {
		.lvl = sys_get_le16(data),
		.new_transaction = true,
		.transition = transition,
	};

	srv->handlers->set(srv, NULL, &set, &status);
}

static void scene_recall_complete(struct bt_mesh_model *model)
{
	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_lvl_status status = { 0 };

	srv->handlers->get(srv, NULL, &status);

	(void)bt_mesh_lvl_srv_pub(srv, NULL, &status);
}

BT_MESH_SCENE_ENTRY_SIG(lvl) = {
	.id.sig = BT_MESH_MODEL_ID_GEN_LEVEL_SRV,
	.maxlen = 2,
	.store = scene_store,
	.recall = scene_recall,
	.recall_complete = scene_recall_complete,
};

static int update_handler(struct bt_mesh_model *model)
{
	struct bt_mesh_lvl_srv *srv = model->user_data;
	struct bt_mesh_lvl_status status = { 0 };

	srv->handlers->get(srv, NULL, &status);
	encode_status(&status, model->pub->msg);

	return 0;
}

static int bt_mesh_lvl_srv_init(struct bt_mesh_model *model)
{
	struct bt_mesh_lvl_srv *srv = model->user_data;

	srv->model = model;
	srv->pub.msg = &srv->pub_buf;
	srv->pub.update = update_handler;
	net_buf_simple_init_with_data(&srv->pub_buf, srv->pub_data,
				      sizeof(srv->pub_data));

	return 0;
}

static void bt_mesh_lvl_srv_reset(struct bt_mesh_model *model)
{
	net_buf_simple_reset(model->pub->msg);
}

const struct bt_mesh_model_cb _bt_mesh_lvl_srv_cb = {
	.init = bt_mesh_lvl_srv_init,
	.reset = bt_mesh_lvl_srv_reset,
};

int bt_mesh_lvl_srv_pub(struct bt_mesh_lvl_srv *srv,
			struct bt_mesh_msg_ctx *ctx,
			const struct bt_mesh_lvl_status *status)
{
	if (!srv->pub.addr) {
		return 0;
	}

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_LVL_OP_STATUS,
				 BT_MESH_LVL_MSG_MAXLEN_STATUS);
	encode_status(status, &msg);

	return model_send(srv->model, ctx, &msg);
}
