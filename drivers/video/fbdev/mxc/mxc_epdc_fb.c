/*
 * Copyright (C) 2010-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
/*
 * Based on STMP378X LCDIF
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

#include <linux/busfreq-imx.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/rwsem.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/ktime.h>
#include <linux/pxp_dma.h>
#include <linux/pm_runtime.h>
#include <linux/mxcfb.h>
#include <linux/mxcfb_epdc.h>
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#include <linux/fsl_devices.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/unaligned.h>


#include "epdc_regs.h"

/*
 * Enable this define to have a default panel
 * loaded during driver initialization
 */
/*#define DEFAULT_PANEL_HW_INIT*/

#define NUM_SCREENS_MIN	3

#define EPDC_V1_NUM_LUTS	16
#define EPDC_V1_MAX_NUM_UPDATES 20
#define EPDC_V2_NUM_LUTS	64
#define EPDC_V2_MAX_NUM_UPDATES 64
#define EPDC_MAX_NUM_BUFFERS	2
#define E70Q10_SOURCE_SNAPSHOT_SLOTS	3
#define E70Q10_SLEEP_BARRIER_TIMEOUT_MS	2500
#define INVALID_LUT		(-1)
#define DRY_RUN_NO_LUT		100

/* Maximum update buffer image width due to v2.0 and v2.1 errata ERR005313. */
#define EPDC_V2_MAX_UPDATE_WIDTH	2047
#define EPDC_V2_ROTATION_ALIGNMENT	8

#define DEFAULT_TEMP_INDEX	0
#define DEFAULT_TEMP		20 /* room temp in deg Celsius */

#define INIT_UPDATE_MARKER	0x12345678
#define PAN_UPDATE_MARKER	0x12345679

#define POWER_STATE_OFF	0
#define POWER_STATE_ON	1

#define MERGE_OK	0
#define MERGE_FAIL	1
#define MERGE_BLOCK	2

/* Netronix histogram thresholds for 4-bit and 5-bit waveform buffers. */
#define EPDC_HIST1_P4N_PARAM	0x00000000
#define EPDC_HIST2_P4N_PARAM	0x00000f00
#define EPDC_HIST4_P4N_PARAM	0x0f0a0500
#define EPDC_HIST8_P4N_PARAM0	0x06040200
#define EPDC_HIST8_P4N_PARAM1	0x0f0d0b09
#define EPDC_HIST16_P4N_PARAM0	0x03020100
#define EPDC_HIST16_P4N_PARAM1	0x07060504
#define EPDC_HIST16_P4N_PARAM2	0x0b0a0908
#define EPDC_HIST16_P4N_PARAM3	0x0f0e0d0c
#define EPDC_HIST1_P5N_PARAM	(EPDC_HIST1_P4N_PARAM << 1)
#define EPDC_HIST2_P5N_PARAM	(EPDC_HIST2_P4N_PARAM << 1)
#define EPDC_HIST4_P5N_PARAM	(EPDC_HIST4_P4N_PARAM << 1)
#define EPDC_HIST8_P5N_PARAM0	(EPDC_HIST8_P4N_PARAM0 << 1)
#define EPDC_HIST8_P5N_PARAM1	(EPDC_HIST8_P4N_PARAM1 << 1)
#define EPDC_HIST16_P5N_PARAM0	(EPDC_HIST16_P4N_PARAM0 << 1)
#define EPDC_HIST16_P5N_PARAM1	(EPDC_HIST16_P4N_PARAM1 << 1)
#define EPDC_HIST16_P5N_PARAM2	(EPDC_HIST16_P4N_PARAM2 << 1)
#define EPDC_HIST16_P5N_PARAM3	(EPDC_HIST16_P4N_PARAM3 << 1)

#define EPDC_QOSC_CCM_CCGR4_OFFSET	0x78
#define EPDC_QOSC_CCM_CCGR4_GATE_MASK	0x30
#define EPDC_QOSC_MASTER_ENABLE_OFFSET	0x40
#define EPDC_QOSC_EPDC_CHANNEL_OFFSET	0x1400
#define EPDC_QOSC_WR_OFFSET		0xd0
#define EPDC_QOSC_RD_OFFSET		0xe0
#define EPDC_QOSC_STATUS_OFFSET		0xf0

static unsigned long default_bpp = 16;
static DEFINE_MUTEX(hard_lock);

struct update_marker_data {
	struct list_head full_list;
	struct list_head upd_list;
	struct list_head sleep_done_list;
	u32 update_marker;
	u32 sleep_sequence;
	u32 update_generation;
	struct completion update_completion;
	int lut_num;
	int result;
	bool collision_test;
	bool waiting;
	bool retained_sleep;
};

struct e70q10_source_snapshot;

struct update_desc_list {
	struct list_head list;
	struct mxcfb_update_data upd_data;/* Update parameters */
	u32 fb_offset;		/* Source FB page captured when update was queued */
	u64 fb_source_generation;
	struct e70q10_source_snapshot *source_snapshot;
	int fb_page_index;
	u64 fb_page_generation;
	bool fb_page_generation_valid;
	u32 epdc_offs;		/* Added to buffer ptr to resolve alignment */
	u32 epdc_stride;	/* Depends on rotation & whether we skip PxP */
	struct list_head upd_marker_list; /* List of markers for this update */
	u32 update_order;	/* Numeric ordering value for update */
	u32 sleep_sequence;	/* Nonzero only for a tagged sleep-screen update */
	bool merged;		/* At least one queued update was merged into this */
	bool collision_resubmit;	/* This descriptor is being retried after collision */
	u32 collided_newer_order;	/* Newer LUT order observed during collision */
	bool collided_newer_seen;
	bool collided_newer_completed_before_wb;
	u32 collision_retry_count;
	bool deferred_latest;	/* Pending source uses the reserved latest slot */
	bool deferred_retry;	/* Was promoted from the reserved latest slot */
};


/* This structure represents a list node containing both
 * a memory region allocated as an output buffer for the PxP
 * update processing task, and the update description (mode, region, etc.) */
struct update_data_list {
	struct list_head list;
	dma_addr_t phys_addr;	/* Pointer to phys address of processed Y buf */
	void *virt_addr;
	struct update_desc_list *update_desc;
	int lut_num;		/* Assigned before update is processed into working buffer */
	bool pxp_used;
	u64 collision_mask;	/* Set when update creates collision */
				/* Mask of the LUTs the update collides with */
};

struct e70q10_source_snapshot {
	void *virt_addr;
	dma_addr_t phys_addr;
	u32 fb_offset;
	u64 generation;
	u32 refs;
};




struct mxc_epdc_fb_data {
	struct fb_info info;
	struct fb_var_screeninfo epdc_fb_var; /* Internal copy of screeninfo
						so we can sync changes to it */
	u32 pseudo_palette[16];
	char fw_str[64];
	struct list_head list;
	struct imx_epdc_fb_mode *cur_mode;
	struct imx_epdc_fb_platform_data *pdata;
	int blank;
	u32 max_pix_size;
	ssize_t map_size;
	dma_addr_t phys_start;
	u32 fb_offset;
	u64 fb_source_generation;
	int default_bpp;
	int native_width;
	int native_height;
	int num_screens;
	bool page_lifetime_diag;
	bool page_lifetime_stats_created;
	u64 *fb_page_generations;
	u32 fb_page_generation_count;
	spinlock_t fb_page_lock;
	u64 source_reused_before_pxp;
	u64 source_reused_before_copybuf;
	u64 untracked_fb_pan;
	u64 untracked_fb_submit;
	bool source_snapshot_enabled;
	struct e70q10_source_snapshot *source_snapshots;
	u32 source_snapshot_count;
	struct e70q10_source_snapshot deferred_latest_snapshot;
	struct update_desc_list *deferred_latest_desc;
	bool deferred_latest_enabled;
	size_t source_snapshot_size;
	size_t source_snapshot_copy_size;
	u64 source_snapshot_copies;
	u64 source_snapshot_hits;
	u64 source_snapshot_busy;
	u64 collision_created;
	u64 collision_retry_submitted;
	u64 collision_retry_dropped_newer;
	u64 collision_retry_zero_mask;
	u64 collision_newer_completed_before_wb;
	u64 collision_retry_max_depth;
	u64 collision_list_peak;
	u64 snapshot_busy_final_candidate;
	u64 deferred_latest_retry_armed;
	u64 deferred_latest_retry_submitted;
	u64 deferred_latest_retry_promoted;
	u64 deferred_latest_retry_success;
	u64 deferred_latest_retry_replaced;
	u64 deferred_latest_retry_dropped_error;
	u64 latest_accepted_generation;
	u64 latest_submitted_generation;
	u64 latest_completed_generation;
	u64 coalesce_passes;
	u64 coalesce_superseded_descs;
	u64 coalesce_superseded_generations;
	u64 coalesce_damage_expands;
	u64 coalesce_full_bounds;
	u64 coalesce_skipped_ineligible;
	u64 coalesce_marker_skips;
	int epdc_irq;
	struct device *dev;
	int power_state;
	int wait_for_powerdown;
	struct completion powerdown_compl;
	struct clk *epdc_clk_axi;
	struct clk *epdc_clk_pix;
	struct regulator *display_regulator;
	struct regulator *vcom_regulator;
	struct regulator *v3p3_regulator;
	bool fw_default_load;
	int rev;
	u32 last_sleep_sequence;
	struct mutex pm_control_mutex;
	bool system_suspend_in_progress;
	bool sleep_barrier_waiting;
	bool sleep_barrier_pending;
	bool sleep_barrier_powerdown_held;
	int sleep_barrier_saved_pwrdown_delay;
	u32 sleep_lut_sequence[64];
	bool display_enabled;
	bool vcom_enabled;
	bool v3p3_enabled;
	bool epdc_clocks_enabled;
	bool runtime_pm_held;
	s32 last_panel_powerup_result;

	/* FB elements related to EPDC updates */
	int num_luts;
	int max_num_updates;
	bool in_init;
	bool hw_ready;
	bool hw_initializing;
	bool waiting_for_idle;
	u32 auto_mode;
	u32 upd_scheme;
	struct list_head upd_pending_list;
	struct list_head upd_buf_queue;
	struct list_head upd_buf_free_list;
	struct list_head upd_buf_collision_list;
	struct update_data_list *cur_update;
	struct mutex queue_mutex;
	bool cleanup_fence_active;
	int cleanup_fence_lut;
	u32 cleanup_fence_marker;
	int trt_entries;
	int temp_index;
	u8 *temp_range_bounds;
	int buf_pix_fmt;
	struct mxcfb_waveform_modes wv_modes;
	bool wv_modes_update;
	u32 *waveform_buffer_virt;
	u32 waveform_buffer_phys;
	u32 waveform_buffer_size;
	u8 *waveform_vcd_buffer;
	size_t waveform_vcd_size;
	u8 *waveform_acd_buffer;
	size_t waveform_acd_size;
	u8 *waveform_xwi_buffer;
	size_t waveform_xwi_size;
	u32 waveform_mc;
	u32 waveform_trc;
	u64 waveform_magic_number;
	u32 *working_buffer_virt;
	u32 working_buffer_phys;
	u32 working_buffer_size;
	dma_addr_t *phys_addr_updbuf;
	void **virt_addr_updbuf;
	u32 upd_buffer_num;
	u32 max_num_buffers;
	dma_addr_t phys_addr_copybuf;	/* Phys address of copied update data */
	void *virt_addr_copybuf;	/* Used for PxP SW workaround */
	u32 order_cnt;
	struct list_head full_marker_list;
	struct list_head sleep_marker_done_list;
	wait_queue_head_t sleep_barrier_waitq;
	atomic_t sleep_barrier_event;
	u32 *lut_update_order;		/* Array size = number of luts */
	u64 epdc_colliding_luts;
	u64 luts_complete_wb;
	struct completion updates_done;
	struct delayed_work epdc_done_work;
	struct workqueue_struct *epdc_submit_workqueue;
	struct work_struct epdc_submit_work;
	struct workqueue_struct *epdc_intr_workqueue;
	struct work_struct epdc_intr_work;
	struct work_struct epdc_recovery_work;
	struct completion epdc_recovery_done;
	spinlock_t epdc_recovery_lock;
	atomic_t epdc_recovery_state;	/* 0=idle, 1=running, 2=failed */
	atomic_t tce_underrun_count;
	int epdc_recovery_result;
	bool suspend_blocked;
	bool recovery_refresh_active;
	bool waiting_for_wb;
	bool waiting_for_lut;
	bool waiting_for_lut15;
	struct completion update_res_free;
	struct completion lut15_free;
	struct completion eof_event;
	int eof_sync_period;
	struct mutex power_mutex;
	bool powering_down;
	/* Keep the first post-init idle powerdown on the legacy boot path. */
	bool boot_initial_powerdown_pending;
	bool updates_active;
	int pwrdown_delay;
	unsigned long tce_prevent;
	bool restrict_width; /* work around rev >=2.0 width and
				stride restriction  */

	/* FB elements related to PxP DMA */
	struct completion pxp_tx_cmpl;
	struct pxp_channel *pxp_chan;
	struct dma_chan *pxp_debug_chan;
	struct pxp_config_data pxp_conf;
	u32 pxp_hist_status;
	dma_cookie_t cookie;
	struct scatterlist sg[2];
	struct mutex pxp_mutex; /* protects access to PxP */
};



static bool chan_filter(struct dma_chan *chan, void *arg);

static struct dma_chan *epdc_request_pxp_pm_channel(void)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);
	return dma_request_channel(mask, chan_filter, NULL);
}



static void epdc_sleep_marker_discard_old_locked(
		struct mxc_epdc_fb_data *fb_data, u32 sequence)
{
	struct update_marker_data *marker, *tmp;

	list_for_each_entry_safe(marker, tmp,
		&fb_data->sleep_marker_done_list, sleep_done_list) {
		if (marker->sleep_sequence == sequence)
			continue;
		list_del_init(&marker->sleep_done_list);
		kfree(marker);
	}
}


static void epdc_sleep_barrier_wake(struct mxc_epdc_fb_data *fb_data)
{
	atomic_inc(&fb_data->sleep_barrier_event);
	wake_up_all(&fb_data->sleep_barrier_waitq);
}



static bool epdc_is_full_panel_gc16_cleanup(
				struct mxc_epdc_fb_data *fb_data,
				const struct mxcfb_update_data *upd);

/* Return a discrete y-pan page index, or -EINVAL for a non-page pan. */
static int epdc_fb_page_index(struct mxc_epdc_fb_data *fb_data, u32 fb_offset)
{
	u32 page_bytes = fb_data->info.fix.line_length * fb_data->info.var.yres;
	u32 page;

	if (!fb_data->page_lifetime_diag || !fb_data->fb_page_generations ||
	    !page_bytes || fb_offset % page_bytes)
		return -EINVAL;

	page = fb_offset / page_bytes;
	if (page >= fb_data->fb_page_generation_count ||
	    (u64)fb_offset + page_bytes > (u64)fb_data->map_size)
		return -EINVAL;

	return page;
}

static bool epdc_fb_page_generation_changed(struct mxc_epdc_fb_data *fb_data,
					   struct update_desc_list *desc)
{
	unsigned long flags;
	bool changed = false;

	if (!fb_data->page_lifetime_diag ||
	    !desc->fb_page_generation_valid ||
	    (desc->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER))
		return false;

	spin_lock_irqsave(&fb_data->fb_page_lock, flags);
	if (desc->fb_page_index >= 0 &&
	    desc->fb_page_index < fb_data->fb_page_generation_count)
		changed = fb_data->fb_page_generations[desc->fb_page_index] !=
			desc->fb_page_generation;
	spin_unlock_irqrestore(&fb_data->fb_page_lock, flags);
	return changed;
}

static void epdc_record_fb_page_reuse(struct mxc_epdc_fb_data *fb_data,
				      bool before_copybuf)
{
	unsigned long flags;

	spin_lock_irqsave(&fb_data->fb_page_lock, flags);
	if (before_copybuf)
		fb_data->source_reused_before_copybuf++;
	else
		fb_data->source_reused_before_pxp++;
	spin_unlock_irqrestore(&fb_data->fb_page_lock, flags);
}

/*
 * E70Q10 queues EPDC work after MXCFB_SEND_UPDATE returns, while Android may
 * recycle the posted framebuffer page immediately because HWC1 has no release
 * fence.  Keep an immutable RGB565 source until the update descriptor becomes
 * terminal.  All helpers below are called with queue_mutex held.
 */
static void e70q10_source_snapshot_put_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct e70q10_source_snapshot *snapshot)
{
	if (!snapshot)
		return;
	if (WARN_ON(snapshot->refs == 0))
		return;
	snapshot->refs--;
	if (snapshot == &fb_data->deferred_latest_snapshot &&
	    snapshot->refs == 0)
		fb_data->deferred_latest_desc = NULL;
}

static void epdc_free_update_desc_locked(struct mxc_epdc_fb_data *fb_data,
					struct update_desc_list *desc)
{
	if (!desc)
		return;
	if (fb_data->deferred_latest_desc == desc) {
		fb_data->deferred_latest_desc = NULL;
		if (desc->deferred_latest)
			fb_data->deferred_latest_retry_replaced++;
	}
	e70q10_source_snapshot_put_locked(fb_data, desc->source_snapshot);
	desc->source_snapshot = NULL;
	kfree(desc);
}

static void e70q10_copy_frame_to_snapshot_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct e70q10_source_snapshot *snapshot, u32 fb_offset,
		u64 generation)
{
	memcpy(snapshot->virt_addr,
	       (u8 *)fb_data->info.screen_base + fb_offset,
	       fb_data->source_snapshot_copy_size);
	if (fb_data->source_snapshot_size > fb_data->source_snapshot_copy_size)
		memset((u8 *)snapshot->virt_addr +
		       fb_data->source_snapshot_copy_size, 0xff,
		       fb_data->source_snapshot_size -
		       fb_data->source_snapshot_copy_size);
	/* Publish CPU stores before PxP is allowed to DMA-read this slot. */
	wmb();
	snapshot->fb_offset = fb_offset;
	snapshot->generation = generation;
}

static bool e70q10_promote_deferred_snapshot_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *desc)
{
	struct e70q10_source_snapshot *slot = NULL;
	u32 i;

	if (!desc || desc->source_snapshot !=
			&fb_data->deferred_latest_snapshot)
		return true;

	for (i = 0; i < fb_data->source_snapshot_count; i++) {
		if (!fb_data->source_snapshots[i].refs) {
			slot = &fb_data->source_snapshots[i];
			break;
		}
	}
	if (!slot)
		return false;

	memcpy(slot->virt_addr,
	       fb_data->deferred_latest_snapshot.virt_addr,
	       fb_data->source_snapshot_copy_size);
	if (fb_data->source_snapshot_size > fb_data->source_snapshot_copy_size)
		memset((u8 *)slot->virt_addr + fb_data->source_snapshot_copy_size,
		       0xff, fb_data->source_snapshot_size -
		       fb_data->source_snapshot_copy_size);
	wmb();
	slot->fb_offset = fb_data->deferred_latest_snapshot.fb_offset;
	slot->generation = fb_data->deferred_latest_snapshot.generation;
	slot->refs = 1;
	fb_data->deferred_latest_snapshot.refs = 0;
	fb_data->deferred_latest_desc = NULL;
	desc->source_snapshot = slot;
	desc->deferred_latest = false;
	desc->deferred_retry = true;
	fb_data->deferred_latest_retry_promoted++;
	fb_data->source_snapshot_copies++;
	return true;
}

static int e70q10_source_snapshot_acquire_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *desc)
{
	struct e70q10_source_snapshot *slot = NULL;
	u32 i;

	if (!fb_data->source_snapshot_enabled ||
	    (desc->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) ||
	    fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT)
		return 0;

	if (!fb_data->source_snapshots || !fb_data->source_snapshot_count ||
	    !fb_data->source_snapshot_copy_size ||
	    (u64)desc->fb_offset + fb_data->source_snapshot_copy_size >
			(u64)fb_data->map_size)
		return -EINVAL;

	/*
	 * Multiple dirty rectangles generated by one fb_post share the same
	 * immutable full-frame copy while at least one descriptor still owns it.
	 * A zero-ref slot is never treated as a cache hit: generic mmap users may
	 * change framebuffer bytes without issuing FBIOPAN_DISPLAY.
	 */
	for (i = 0; i < fb_data->source_snapshot_count; i++) {
		struct e70q10_source_snapshot *candidate =
			&fb_data->source_snapshots[i];

		if (candidate->refs &&
		    candidate->fb_offset == desc->fb_offset &&
		    candidate->generation == desc->fb_source_generation) {
			candidate->refs++;
			desc->source_snapshot = candidate;
			fb_data->source_snapshot_hits++;
			return 0;
		}
		if (!candidate->refs && !slot)
			slot = candidate;
	}

	if (!slot) {
		return -EBUSY;
	}

	e70q10_copy_frame_to_snapshot_locked(fb_data, slot,
		desc->fb_offset, desc->fb_source_generation);
	slot->refs = 1;
	desc->source_snapshot = slot;
	fb_data->source_snapshot_copies++;
	return 0;
}

static void e70q10_source_snapshot_adopt_newer_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *dst,
		struct update_desc_list *src)
{
	if (dst->source_snapshot == src->source_snapshot)
		return;

	e70q10_source_snapshot_put_locked(fb_data, dst->source_snapshot);
	dst->source_snapshot = src->source_snapshot;
	/* Move src's reference; its descriptor will be freed after the merge. */
	src->source_snapshot = NULL;
}

struct waveform_data_header {
	unsigned int wi0;
	unsigned int wi1;
	unsigned int wi2;
	unsigned int wi3;
	unsigned int wi4;
	unsigned int wi5;
	unsigned int wi6;
	unsigned int xwia:24;
	unsigned int cs1:8;
	unsigned int wmta:24;
	unsigned int fvsn:8;
	unsigned int luts:8;
	unsigned int mc:8;
	unsigned int trc:8;
	unsigned int awv:8;
	unsigned int eb:8;
	unsigned int sb:8;
	unsigned int reserved0_1:8;
	unsigned int reserved0_2:8;
	unsigned int reserved0_3:8;
	unsigned int reserved0_4:8;
	unsigned int reserved0_5:8;
	unsigned int cs2:8;
};

struct mxcfb_waveform_data_file {
	struct waveform_data_header wdh;
	u32 *data;	/* Temperature Range Table + Waveform Data */
};

/*
 * Netronix generation-2 waveform files append voltage-control (VCD),
 * algorithm-control (ACD), and optional textual (XWI) sections after the
 * waveform proper.  Their offsets are relative to the first mode-offset
 * table byte.  The EPDC DMA buffer must end before those appended sections.
 */
static int netronix_parse_advanced_waveform(struct mxc_epdc_fb_data *fb_data,
		const struct waveform_data_header *wdh, const u8 *data,
		size_t data_size, size_t *core_size)
{
	size_t table_size = 8 * ((size_t)wdh->mc + 1);
	size_t meta_size = 0;
	u64 avc = 0, acd = 0, xwi = 0;
	size_t tail;

	if (data_size < table_size || table_size < 8)
		return -EINVAL;

	/* Old waveform files have no metadata after the mode-offset table. */
	if (get_unaligned_le64(data) <= table_size)
		return 0;

	switch (wdh->awv) {
	case 0:
		meta_size = wdh->xwia ? 8 : 0;
		if (wdh->xwia && data_size >= table_size + meta_size)
			xwi = get_unaligned_le64(data + table_size);
		break;
	case 1:
		meta_size = wdh->xwia ? 16 : 8;
		if (data_size >= table_size + meta_size) {
			avc = get_unaligned_le64(data + table_size);
			if (wdh->xwia)
				xwi = get_unaligned_le64(data + table_size + 8);
		}
		break;
	case 2:
		meta_size = wdh->xwia ? 24 : 16;
		if (data_size >= table_size + meta_size) {
			acd = get_unaligned_le64(data + table_size);
			fb_data->waveform_magic_number =
				get_unaligned_le64(data + table_size + 8);
			if (wdh->xwia)
				xwi = get_unaligned_le64(data + table_size + 16);
		}
		break;
	case 3:
		meta_size = wdh->xwia ? 32 : 24;
		if (data_size >= table_size + meta_size) {
			avc = get_unaligned_le64(data + table_size);
			acd = get_unaligned_le64(data + table_size + 8);
			fb_data->waveform_magic_number =
				get_unaligned_le64(data + table_size + 16);
			if (wdh->xwia)
				xwi = get_unaligned_le64(data + table_size + 24);
		}
		break;
	default:
		return -EINVAL;
	}

	if (data_size < table_size + meta_size)
		return -EINVAL;
	if ((avc && (avc < table_size + meta_size || avc > data_size)) ||
	    (acd && (acd < table_size + meta_size || acd > data_size)) ||
	    (xwi && (xwi < table_size + meta_size || xwi > data_size)) ||
	    (avc && acd && avc > acd) || (acd && xwi && acd > xwi) ||
	    (avc && xwi && avc > xwi))
		return -EINVAL;

	tail = xwi ? (size_t)xwi : data_size;
	if (avc) {
		size_t end = acd ? (size_t)acd : tail;

		fb_data->waveform_vcd_size = end - (size_t)avc;
		fb_data->waveform_vcd_buffer = kmemdup(data + avc,
			fb_data->waveform_vcd_size, GFP_KERNEL);
		if (!fb_data->waveform_vcd_buffer)
			goto nomem;
	}
	if (acd) {
		fb_data->waveform_acd_size = tail - (size_t)acd;
		fb_data->waveform_acd_buffer = kmemdup(data + acd,
			fb_data->waveform_acd_size, GFP_KERNEL);
		if (!fb_data->waveform_acd_buffer)
			goto nomem;
	}
	if (xwi) {
		fb_data->waveform_xwi_size = data_size - (size_t)xwi;
		fb_data->waveform_xwi_buffer = kmemdup(data + xwi,
			fb_data->waveform_xwi_size, GFP_KERNEL);
		if (!fb_data->waveform_xwi_buffer)
			goto nomem;
	}

	if (avc)
		*core_size = (size_t)avc;
	else if (acd)
		*core_size = (size_t)acd;
	else if (xwi)
		*core_size = (size_t)xwi;

	fb_data->waveform_mc = wdh->mc + 1;
	fb_data->waveform_trc = wdh->trc + 1;
	dev_info(fb_data->dev,
		 "Netronix advanced waveform awv=%u: core=%zu VCD=%zu ACD=%zu XWI=%zu\n",
		 wdh->awv, *core_size, fb_data->waveform_vcd_size,
		 fb_data->waveform_acd_size, fb_data->waveform_xwi_size);
	return 0;

nomem:
	kfree(fb_data->waveform_vcd_buffer);
	kfree(fb_data->waveform_acd_buffer);
	kfree(fb_data->waveform_xwi_buffer);
	fb_data->waveform_vcd_buffer = NULL;
	fb_data->waveform_acd_buffer = NULL;
	fb_data->waveform_xwi_buffer = NULL;
	fb_data->waveform_vcd_size = 0;
	fb_data->waveform_acd_size = 0;
	fb_data->waveform_xwi_size = 0;
	return -ENOMEM;
}

static struct fb_videomode ed078kh1_mode = {
	.name = "ED078KH1",
	.refresh = 85,
	.xres = 1872,
	.yres = 1404,
	.pixclock = 133400000,
	.left_margin = 44,
	.right_margin = 89,
	.upper_margin = 4,
	.lower_margin = 5,
	.hsync_len = 44,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e060scm_mode = {
	.name = "E060SCM",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 26666667,
	.left_margin = 8,
	.right_margin = 100,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct imx_epdc_fb_mode panel_modes[] = {
	{
		&ed078kh1_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		772,    /* gdclk_hp_offs */
		757,    /* gdsp_offs */
		0,      /* gdoe_offs */
		199,    /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		465,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		9,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e060scm_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		419,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		5,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	}
};

static struct imx_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
};

void __iomem *epdc_base;

struct mxc_epdc_fb_data *g_fb_data;

/* forward declaration */
static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data,
						int temp);
static int mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data);
static int epdc_wait_for_hw_idle(struct mxc_epdc_fb_data *fb_data,
				 bool allow_powering_down,
				 unsigned int timeout_ms,
				 u32 *pxp_regs);
static int epdc_hw_idle_locked(struct mxc_epdc_fb_data *fb_data,
			       bool allow_powering_down);
static int mxc_epdc_fb_blank(int blank, struct fb_info *info);
static int __mxc_epdc_fb_send_single_update(
	struct mxcfb_update_data *upd_data, struct fb_info *info,
	bool serialize, void *serialize_wait_state, u32 sleep_sequence);
static void epdc_recovery_work_func(struct work_struct *work);
static int mxc_epdc_fb_init_hw(struct fb_info *info);
static void epdc_sleep_barrier_restore_powerdown_locked(
	struct mxc_epdc_fb_data *fb_data, bool schedule_if_idle);
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region,
			      struct update_desc_list *source_desc);
static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat);

static void draw_mode0(struct mxc_epdc_fb_data *fb_data);
static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data);

static void do_dithering_processing_Y1_v1_0(
		unsigned char *update_region_virt_ptr,
		dma_addr_t update_region_phys_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist);
static void do_dithering_processing_Y4_v1_0(
		unsigned char *update_region_virt_ptr,
		dma_addr_t update_region_phys_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist);

#ifdef DEBUG
static void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
			    struct pxp_config_data *pxp_conf)
{
	dev_info(fb_data->dev, "S0 fmt 0x%x",
		pxp_conf->s0_param.pixel_fmt);
	dev_info(fb_data->dev, "S0 width 0x%x",
		pxp_conf->s0_param.width);
	dev_info(fb_data->dev, "S0 height 0x%x",
		pxp_conf->s0_param.height);
	dev_info(fb_data->dev, "S0 ckey 0x%x",
		pxp_conf->s0_param.color_key);
	dev_info(fb_data->dev, "S0 ckey en 0x%x",
		pxp_conf->s0_param.color_key_enable);

	dev_info(fb_data->dev, "OL0 combine en 0x%x",
		pxp_conf->ol_param[0].combine_enable);
	dev_info(fb_data->dev, "OL0 fmt 0x%x",
		pxp_conf->ol_param[0].pixel_fmt);
	dev_info(fb_data->dev, "OL0 width 0x%x",
		pxp_conf->ol_param[0].width);
	dev_info(fb_data->dev, "OL0 height 0x%x",
		pxp_conf->ol_param[0].height);
	dev_info(fb_data->dev, "OL0 ckey 0x%x",
		pxp_conf->ol_param[0].color_key);
	dev_info(fb_data->dev, "OL0 ckey en 0x%x",
		pxp_conf->ol_param[0].color_key_enable);
	dev_info(fb_data->dev, "OL0 alpha 0x%x",
		pxp_conf->ol_param[0].global_alpha);
	dev_info(fb_data->dev, "OL0 alpha en 0x%x",
		pxp_conf->ol_param[0].global_alpha_enable);
	dev_info(fb_data->dev, "OL0 local alpha en 0x%x",
		pxp_conf->ol_param[0].local_alpha_enable);

	dev_info(fb_data->dev, "Out fmt 0x%x",
		pxp_conf->out_param.pixel_fmt);
	dev_info(fb_data->dev, "Out width 0x%x",
		pxp_conf->out_param.width);
	dev_info(fb_data->dev, "Out height 0x%x",
		pxp_conf->out_param.height);

	dev_info(fb_data->dev,
		"drect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.drect.left, pxp_conf->proc_data.drect.top,
		pxp_conf->proc_data.drect.width,
		pxp_conf->proc_data.drect.height);
	dev_info(fb_data->dev,
		"srect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.srect.left, pxp_conf->proc_data.srect.top,
		pxp_conf->proc_data.srect.width,
		pxp_conf->proc_data.srect.height);
	dev_info(fb_data->dev, "Scaling en 0x%x", pxp_conf->proc_data.scaling);
	dev_info(fb_data->dev, "HFlip en 0x%x", pxp_conf->proc_data.hflip);
	dev_info(fb_data->dev, "VFlip en 0x%x", pxp_conf->proc_data.vflip);
	dev_info(fb_data->dev, "Rotation 0x%x", pxp_conf->proc_data.rotate);
	dev_info(fb_data->dev, "BG Color 0x%x", pxp_conf->proc_data.bgcolor);
}

static void dump_epdc_reg(void)
{
	printk(KERN_DEBUG "\n\n");
	printk(KERN_DEBUG "EPDC_CTRL 0x%x\n", __raw_readl(EPDC_CTRL));
	printk(KERN_DEBUG "EPDC_WVADDR 0x%x\n", __raw_readl(EPDC_WVADDR));
	printk(KERN_DEBUG "EPDC_WB_ADDR 0x%x\n", __raw_readl(EPDC_WB_ADDR));
	printk(KERN_DEBUG "EPDC_RES 0x%x\n", __raw_readl(EPDC_RES));
	printk(KERN_DEBUG "EPDC_FORMAT 0x%x\n", __raw_readl(EPDC_FORMAT));
	printk(KERN_DEBUG "EPDC_FIFOCTRL 0x%x\n", __raw_readl(EPDC_FIFOCTRL));
	printk(KERN_DEBUG "EPDC_UPD_ADDR 0x%x\n", __raw_readl(EPDC_UPD_ADDR));
	printk(KERN_DEBUG "EPDC_UPD_STRIDE 0x%x\n", __raw_readl(EPDC_UPD_STRIDE));
	printk(KERN_DEBUG "EPDC_UPD_FIXED 0x%x\n", __raw_readl(EPDC_UPD_FIXED));
	printk(KERN_DEBUG "EPDC_UPD_CORD 0x%x\n", __raw_readl(EPDC_UPD_CORD));
	printk(KERN_DEBUG "EPDC_UPD_SIZE 0x%x\n", __raw_readl(EPDC_UPD_SIZE));
	printk(KERN_DEBUG "EPDC_UPD_CTRL 0x%x\n", __raw_readl(EPDC_UPD_CTRL));
	printk(KERN_DEBUG "EPDC_TEMP 0x%x\n", __raw_readl(EPDC_TEMP));
	printk(KERN_DEBUG "EPDC_AUTOWV_LUT 0x%x\n", __raw_readl(EPDC_AUTOWV_LUT));
	printk(KERN_DEBUG "EPDC_TCE_CTRL 0x%x\n", __raw_readl(EPDC_TCE_CTRL));
	printk(KERN_DEBUG "EPDC_TCE_SDCFG 0x%x\n", __raw_readl(EPDC_TCE_SDCFG));
	printk(KERN_DEBUG "EPDC_TCE_GDCFG 0x%x\n", __raw_readl(EPDC_TCE_GDCFG));
	printk(KERN_DEBUG "EPDC_TCE_HSCAN1 0x%x\n", __raw_readl(EPDC_TCE_HSCAN1));
	printk(KERN_DEBUG "EPDC_TCE_HSCAN2 0x%x\n", __raw_readl(EPDC_TCE_HSCAN2));
	printk(KERN_DEBUG "EPDC_TCE_VSCAN 0x%x\n", __raw_readl(EPDC_TCE_VSCAN));
	printk(KERN_DEBUG "EPDC_TCE_OE 0x%x\n", __raw_readl(EPDC_TCE_OE));
	printk(KERN_DEBUG "EPDC_TCE_POLARITY 0x%x\n", __raw_readl(EPDC_TCE_POLARITY));
	printk(KERN_DEBUG "EPDC_TCE_TIMING1 0x%x\n", __raw_readl(EPDC_TCE_TIMING1));
	printk(KERN_DEBUG "EPDC_TCE_TIMING2 0x%x\n", __raw_readl(EPDC_TCE_TIMING2));
	printk(KERN_DEBUG "EPDC_TCE_TIMING3 0x%x\n", __raw_readl(EPDC_TCE_TIMING3));
	printk(KERN_DEBUG "EPDC_PIGEON_CTRL0 0x%x\n", __raw_readl(EPDC_PIGEON_CTRL0));
	printk(KERN_DEBUG "EPDC_PIGEON_CTRL1 0x%x\n", __raw_readl(EPDC_PIGEON_CTRL1));
	printk(KERN_DEBUG "EPDC_IRQ_MASK1 0x%x\n", __raw_readl(EPDC_IRQ_MASK1));
	printk(KERN_DEBUG "EPDC_IRQ_MASK2 0x%x\n", __raw_readl(EPDC_IRQ_MASK2));
	printk(KERN_DEBUG "EPDC_IRQ1 0x%x\n", __raw_readl(EPDC_IRQ1));
	printk(KERN_DEBUG "EPDC_IRQ2 0x%x\n", __raw_readl(EPDC_IRQ2));
	printk(KERN_DEBUG "EPDC_IRQ_MASK 0x%x\n", __raw_readl(EPDC_IRQ_MASK));
	printk(KERN_DEBUG "EPDC_IRQ 0x%x\n", __raw_readl(EPDC_IRQ));
	printk(KERN_DEBUG "EPDC_STATUS_LUTS 0x%x\n", __raw_readl(EPDC_STATUS_LUTS));
	printk(KERN_DEBUG "EPDC_STATUS_LUTS2 0x%x\n", __raw_readl(EPDC_STATUS_LUTS2));
	printk(KERN_DEBUG "EPDC_STATUS_NEXTLUT 0x%x\n", __raw_readl(EPDC_STATUS_NEXTLUT));
	printk(KERN_DEBUG "EPDC_STATUS_COL1 0x%x\n", __raw_readl(EPDC_STATUS_COL));
	printk(KERN_DEBUG "EPDC_STATUS_COL2 0x%x\n", __raw_readl(EPDC_STATUS_COL2));
	printk(KERN_DEBUG "EPDC_STATUS 0x%x\n", __raw_readl(EPDC_STATUS));
	printk(KERN_DEBUG "EPDC_UPD_COL_CORD 0x%x\n", __raw_readl(EPDC_UPD_COL_CORD));
	printk(KERN_DEBUG "EPDC_UPD_COL_SIZE 0x%x\n", __raw_readl(EPDC_UPD_COL_SIZE));
	printk(KERN_DEBUG "EPDC_DEBUG 0x%x\n", __raw_readl(EPDC_DEBUG));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT 0x%x\n", __raw_readl(EPDC_DEBUG_LUT));
	printk(KERN_DEBUG "EPDC_HIST1_PARAM 0x%x\n", __raw_readl(EPDC_HIST1_PARAM));
	printk(KERN_DEBUG "EPDC_HIST2_PARAM 0x%x\n", __raw_readl(EPDC_HIST2_PARAM));
	printk(KERN_DEBUG "EPDC_HIST4_PARAM 0x%x\n", __raw_readl(EPDC_HIST4_PARAM));
	printk(KERN_DEBUG "EPDC_HIST8_PARAM0 0x%x\n", __raw_readl(EPDC_HIST8_PARAM0));
	printk(KERN_DEBUG "EPDC_HIST8_PARAM1 0x%x\n", __raw_readl(EPDC_HIST8_PARAM1));
	printk(KERN_DEBUG "EPDC_HIST16_PARAM0 0x%x\n", __raw_readl(EPDC_HIST16_PARAM0));
	printk(KERN_DEBUG "EPDC_HIST16_PARAM1 0x%x\n", __raw_readl(EPDC_HIST16_PARAM1));
	printk(KERN_DEBUG "EPDC_HIST16_PARAM2 0x%x\n", __raw_readl(EPDC_HIST16_PARAM2));
	printk(KERN_DEBUG "EPDC_HIST16_PARAM3 0x%x\n", __raw_readl(EPDC_HIST16_PARAM3));
	printk(KERN_DEBUG "EPDC_GPIO 0x%x\n", __raw_readl(EPDC_GPIO));
	printk(KERN_DEBUG "EPDC_VERSION 0x%x\n", __raw_readl(EPDC_VERSION));
	printk(KERN_DEBUG "\n\n");
}

static void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list)
{
	dev_info(dev,
		"X = %d, Y = %d, Width = %d, Height = %d, WaveMode = %d, "
		"LUT = %d, Coll Mask = 0x%llx, order = %d\n",
		upd_data_list->update_desc->upd_data.update_region.left,
		upd_data_list->update_desc->upd_data.update_region.top,
		upd_data_list->update_desc->upd_data.update_region.width,
		upd_data_list->update_desc->upd_data.update_region.height,
		upd_data_list->update_desc->upd_data.waveform_mode,
		upd_data_list->lut_num,
		upd_data_list->collision_mask,
		upd_data_list->update_desc->update_order);
}

static void dump_collision_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_info(fb_data->dev, "Collision List:\n");
	if (list_empty(&fb_data->upd_buf_collision_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_collision_list, list) {
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_free_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_info(fb_data->dev, "Free List:\n");
	if (list_empty(&fb_data->upd_buf_free_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
}

static void dump_queue(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_info(fb_data->dev, "Queue:\n");
	if (list_empty(&fb_data->upd_buf_queue))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_queue, list) {
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_desc_data(struct device *dev,
			     struct update_desc_list *upd_desc_list)
{
	dev_info(dev,
		"X = %d, Y = %d, Width = %d, Height = %d, WaveMode = %d, "
		"order = %d\n",
		upd_desc_list->upd_data.update_region.left,
		upd_desc_list->upd_data.update_region.top,
		upd_desc_list->upd_data.update_region.width,
		upd_desc_list->upd_data.update_region.height,
		upd_desc_list->upd_data.waveform_mode,
		upd_desc_list->update_order);
}

static void dump_pending_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_desc_list *plist;

	dev_info(fb_data->dev, "Queue:\n");
	if (list_empty(&fb_data->upd_pending_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_pending_list, list)
		dump_desc_data(fb_data->dev, plist);
}

static void dump_all_updates(struct mxc_epdc_fb_data *fb_data)
{
	dump_free_list(fb_data);
	dump_queue(fb_data);
	dump_collision_list(fb_data);
	dev_info(fb_data->dev, "Current update being processed:\n");
	if (fb_data->cur_update == NULL)
		dev_info(fb_data->dev, "No current update\n");
	else
		dump_update_data(fb_data->dev, fb_data->cur_update);
}
#else
static inline void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
				   struct pxp_config_data *pxp_conf) {}
static inline void dump_epdc_reg(void) {}
static inline void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list) {}
static inline void dump_collision_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_free_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_queue(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_all_updates(struct mxc_epdc_fb_data *fb_data) {}

#endif


/********************************************************
 * Start Low-Level EPDC Functions
 ********************************************************/

static inline void epdc_lut_complete_intr(int rev, u32 lut_num, bool enable)
{
	if (rev < 20) {
		if (enable)
			__raw_writel(1 << lut_num, EPDC_IRQ_MASK_SET);
		else
			__raw_writel(1 << lut_num, EPDC_IRQ_MASK_CLEAR);
	} else {
		if (enable) {
			if (lut_num < 32)
				__raw_writel(1 << lut_num, EPDC_IRQ_MASK1_SET);
			else
				__raw_writel(1 << (lut_num - 32),
					EPDC_IRQ_MASK2_SET);
		} else {
			if (lut_num < 32)
				__raw_writel(1 << lut_num,
					EPDC_IRQ_MASK1_CLEAR);
			else
				__raw_writel(1 << (lut_num - 32),
					EPDC_IRQ_MASK2_CLEAR);
		}
	}
}

static inline void epdc_working_buf_intr(bool enable)
{
	if (enable)
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_clear_working_buf_irq(void)
{
	__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ | EPDC_IRQ_LUT_COL_IRQ,
		     EPDC_IRQ_CLEAR);
}

static inline void epdc_eof_intr(bool enable)
{
	if (enable)
		__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_clear_eof_irq(void)
{
	__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_CLEAR);
}

static inline bool epdc_signal_eof(void)
{
	return (__raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ)
		& EPDC_IRQ_FRAME_END_IRQ) ? true : false;
}

static inline void epdc_set_temp(u32 temp)
{
	__raw_writel(temp, EPDC_TEMP);
}

static inline void epdc_set_screen_res(u32 width, u32 height)
{
	u32 val = (height << EPDC_RES_VERTICAL_OFFSET) | width;
	__raw_writel(val, EPDC_RES);
}

static inline void epdc_set_update_addr(u32 addr)
{
	__raw_writel(addr, EPDC_UPD_ADDR);
}

static inline void epdc_set_update_coord(u32 x, u32 y)
{
	u32 val = (y << EPDC_UPD_CORD_YCORD_OFFSET) | x;
	__raw_writel(val, EPDC_UPD_CORD);
}

static inline void epdc_set_update_dimensions(u32 width, u32 height)
{
	u32 val = (height << EPDC_UPD_SIZE_HEIGHT_OFFSET) | width;
	__raw_writel(val, EPDC_UPD_SIZE);
}

static void epdc_set_update_waveform(struct mxcfb_waveform_modes *wv_modes)
{
	u32 val;

	/* Configure the auto-waveform look-up table based on waveform modes */

	/* Entry 1 = DU, 2 = GC4, 3 = GC8, etc. */
	val = (wv_modes->mode_du << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(0 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_du << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(1 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc4 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(2 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc8 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(3 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc16 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(4 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc32 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(5 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
}

static void epdc_set_update_stride(u32 stride)
{
	__raw_writel(stride, EPDC_UPD_STRIDE);
}

static void epdc_submit_update(struct mxc_epdc_fb_data *fb_data,
				u32 lut_num, u32 waveform_mode, u32 update_mode,
			       bool use_dry_run, bool use_test_mode, u32 np_val)
{
	u32 reg_val = 0;


	if (use_test_mode) {
		reg_val |=
		    ((np_val << EPDC_UPD_FIXED_FIXNP_OFFSET) &
		     EPDC_UPD_FIXED_FIXNP_MASK) | EPDC_UPD_FIXED_FIXNP_EN;

		__raw_writel(reg_val, EPDC_UPD_FIXED);

		reg_val = EPDC_UPD_CTRL_USE_FIXED;
	} else {
		__raw_writel(reg_val, EPDC_UPD_FIXED);
	}

	if (waveform_mode == WAVEFORM_MODE_AUTO)
		reg_val |= EPDC_UPD_CTRL_AUTOWV;
	else
		reg_val |= ((waveform_mode <<
				EPDC_UPD_CTRL_WAVEFORM_MODE_OFFSET) &
				EPDC_UPD_CTRL_WAVEFORM_MODE_MASK);

	reg_val |= (use_dry_run ? EPDC_UPD_CTRL_DRY_RUN : 0) |
	    ((lut_num << EPDC_UPD_CTRL_LUT_SEL_OFFSET) &
	     EPDC_UPD_CTRL_LUT_SEL_MASK) |
	    update_mode;

	__raw_writel(reg_val, EPDC_UPD_CTRL);
}

static inline bool epdc_is_lut_complete(int rev, u32 lut_num)
{
	u32 val;
	bool is_compl;
	if (rev < 20) {
		val = __raw_readl(EPDC_IRQ);
		is_compl = val & (1 << lut_num) ? true : false;
	} else if (lut_num < 32) {
		val = __raw_readl(EPDC_IRQ1);
		is_compl = val & (1 << lut_num) ? true : false;
	} else {
		val = __raw_readl(EPDC_IRQ2);
		is_compl = val & (1 << (lut_num - 32)) ? true : false;
	}

	return is_compl;
}

static inline void epdc_clear_lut_complete_irq(int rev, u32 lut_num)
{
	if (rev < 20)
		__raw_writel(1 << lut_num, EPDC_IRQ_CLEAR);
	else if (lut_num < 32)
		__raw_writel(1 << lut_num, EPDC_IRQ1_CLEAR);
	else
		__raw_writel(1 << (lut_num - 32), EPDC_IRQ2_CLEAR);
}

static inline bool epdc_is_lut_active(u32 lut_num)
{
	u32 val;
	bool is_active;

	if (lut_num < 32) {
		val = __raw_readl(EPDC_STATUS_LUTS);
		is_active = val & (1 << lut_num) ? true : false;
	} else {
		val = __raw_readl(EPDC_STATUS_LUTS2);
		is_active = val & (1 << (lut_num - 32)) ? true : false;
	}

	return is_active;
}

static inline bool epdc_any_luts_active(int rev)
{
	bool any_active;

	if (rev < 20)
		any_active = __raw_readl(EPDC_STATUS_LUTS) ? true : false;
	else
		any_active = (__raw_readl(EPDC_STATUS_LUTS) |
			__raw_readl(EPDC_STATUS_LUTS2))	? true : false;

	return any_active;
}

static inline bool epdc_any_luts_available(void)
{
	bool luts_available =
	    (__raw_readl(EPDC_STATUS_NEXTLUT) &
	     EPDC_STATUS_NEXTLUT_NEXT_LUT_VALID) ? true : false;
	return luts_available;
}

static inline int epdc_get_next_lut(void)
{
	u32 val =
	    __raw_readl(EPDC_STATUS_NEXTLUT) &
	    EPDC_STATUS_NEXTLUT_NEXT_LUT_MASK;
	return val;
}

static int epdc_choose_next_lut(int rev, int *next_lut)
{
	u64 luts_status, unprocessed_luts, used_luts;
	/* Available LUTs are reduced to 16 in 5-bit waveform mode */
	bool format_p5n = ((__raw_readl(EPDC_FORMAT) &
	EPDC_FORMAT_BUF_PIXEL_FORMAT_MASK) ==
	EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N);

	luts_status = __raw_readl(EPDC_STATUS_LUTS);
	if ((rev < 20) || format_p5n)
		luts_status &= 0xFFFF;
	else
		luts_status |= ((u64)__raw_readl(EPDC_STATUS_LUTS2) << 32);

	if (rev < 20) {
		unprocessed_luts = __raw_readl(EPDC_IRQ) & 0xFFFF;
	} else {
		unprocessed_luts = __raw_readl(EPDC_IRQ1) |
			((u64)__raw_readl(EPDC_IRQ2) << 32);
		if (format_p5n)
			unprocessed_luts &= 0xFFFF;
	}

	/*
	 * Note on unprocessed_luts: There is a race condition
	 * where a LUT completes, but has not been processed by
	 * IRQ handler workqueue, and then a new update request
	 * attempts to use that LUT.  We prevent that here by
	 * ensuring that the LUT we choose doesn't have its IRQ
	 * bit set (indicating it has completed but not yet been
	 * processed).
	 */
	used_luts = luts_status | unprocessed_luts;

	/*
	 * Selecting a LUT to minimize incidence of TCE Underrun Error
	 * --------------------------------------------------------
	 * We want to find the lowest order LUT that is of greater
	 * order than all other active LUTs.  If highest order LUT
	 * is active, then we want to choose the lowest order
	 * available LUT.
	 *
	 * NOTE: For EPDC version 2.0 and later, TCE Underrun error
	 *       bug is fixed, so it doesn't matter which LUT is used.
	 */

	if ((rev < 20) || format_p5n) {
		*next_lut = fls64(used_luts);
		if (*next_lut > 15)
			*next_lut = ffz(used_luts);
	} else {
		if ((u32)used_luts != ~0UL)
			*next_lut = ffz((u32)used_luts);
		else if ((u32)(used_luts >> 32) != ~0UL)
			*next_lut = ffz((u32)(used_luts >> 32)) + 32;
		else
			*next_lut = INVALID_LUT;
	}

	if (used_luts & 0x8000)
		return 1;
	else
		return 0;
}

static inline bool epdc_is_working_buffer_busy(void)
{
	u32 val = __raw_readl(EPDC_STATUS);
	bool is_busy = (val & EPDC_STATUS_WB_BUSY) ? true : false;

	return is_busy;
}

static inline bool epdc_is_working_buffer_complete(void)
{
	u32 val = __raw_readl(EPDC_IRQ);
	bool is_compl = (val & EPDC_IRQ_WB_CMPLT_IRQ) ? true : false;

	return is_compl;
}

static inline bool epdc_is_lut_cancelled(void)
{
	u32 val = __raw_readl(EPDC_STATUS);
	bool is_void = (val & EPDC_STATUS_UPD_VOID) ? true : false;

	return is_void;
}

static inline bool epdc_is_collision(void)
{
	u32 val = __raw_readl(EPDC_IRQ);
	return (val & EPDC_IRQ_LUT_COL_IRQ) ? true : false;
}

static inline u64 epdc_get_colliding_luts(int rev)
{
	u32 val = __raw_readl(EPDC_STATUS_COL);
	if (rev >= 20)
		val |= (u64)__raw_readl(EPDC_STATUS_COL2) << 32;
	return val;
}

static void epdc_set_horizontal_timing(u32 horiz_start, u32 horiz_end,
				       u32 hsync_width, u32 hsync_line_length)
{
	u32 reg_val =
	    ((hsync_width << EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_OFFSET) &
	     EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_MASK)
	    | ((hsync_line_length << EPDC_TCE_HSCAN1_LINE_SYNC_OFFSET) &
	       EPDC_TCE_HSCAN1_LINE_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN1);

	reg_val =
	    ((horiz_start << EPDC_TCE_HSCAN2_LINE_BEGIN_OFFSET) &
	     EPDC_TCE_HSCAN2_LINE_BEGIN_MASK)
	    | ((horiz_end << EPDC_TCE_HSCAN2_LINE_END_OFFSET) &
	       EPDC_TCE_HSCAN2_LINE_END_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN2);
}

static void epdc_set_vertical_timing(u32 vert_start, u32 vert_end,
				    u32 vsync_width)
{
	u32 reg_val =
	    ((vert_start << EPDC_TCE_VSCAN_FRAME_BEGIN_OFFSET) &
	     EPDC_TCE_VSCAN_FRAME_BEGIN_MASK)
	    | ((vert_end << EPDC_TCE_VSCAN_FRAME_END_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_END_MASK)
	    | ((vsync_width << EPDC_TCE_VSCAN_FRAME_SYNC_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_VSCAN);
}

static void epdc_program_settings(struct mxc_epdc_fb_data *fb_data)
{
	struct imx_epdc_fb_mode *epdc_mode = fb_data->cur_mode;
	u32 reg_val;
	int num_ce;
	int i;

	/* EPDC_CTRL */
	reg_val = __raw_readl(EPDC_CTRL);
	reg_val &= ~EPDC_CTRL_UPD_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_UPD_DATA_SWIZZLE_NO_SWAP;
	reg_val &= ~EPDC_CTRL_LUT_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_LUT_DATA_SWIZZLE_NO_SWAP;
	__raw_writel(reg_val, EPDC_CTRL_SET);

	/* EPDC_FORMAT follows the 4/5-bit format declared by the waveform. */
	reg_val = EPDC_FORMAT_TFT_PIXEL_FORMAT_2BIT
	    | fb_data->buf_pix_fmt
	    | ((0x0 << EPDC_FORMAT_DEFAULT_TFT_PIXEL_OFFSET) &
	       EPDC_FORMAT_DEFAULT_TFT_PIXEL_MASK);
	__raw_writel(reg_val, EPDC_FORMAT);

	/* Netronix high-resolution FIFO thresholds prevent TCE underruns. */
	reg_val = EPDC_FIFOCTRL_ENABLE_PRIORITY |
	    ((200 << EPDC_FIFOCTRL_FIFO_INIT_LEVEL_OFFSET) &
	     EPDC_FIFOCTRL_FIFO_INIT_LEVEL_MASK)
	    | ((255 << EPDC_FIFOCTRL_FIFO_H_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_H_LEVEL_MASK)
	    | ((254 << EPDC_FIFOCTRL_FIFO_L_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_L_LEVEL_MASK);
	__raw_writel(reg_val, EPDC_FIFOCTRL);

	/* EPDC_TEMP - Use default temp to get index */
	epdc_set_temp(fb_data->temp_index =
		mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP));

	/* EPDC_RES */
	epdc_set_screen_res(epdc_mode->vmode->xres, epdc_mode->vmode->yres);

	/* EPDC_AUTOWV_LUT */
	/* Initialize all auto-wavefrom look-up values to 2 - GC16 */
	for (i = 0; i < 8; i++)
		__raw_writel((2 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
			(i << EPDC_AUTOWV_LUT_ADDR_OFFSET), EPDC_AUTOWV_LUT);

	/*
	 * EPDC_TCE_CTRL
	 * VSCAN_HOLDOFF defaults to the selected panel mode's value (4 for
	 * E70Q10).
	 * VCOM_MODE = MANUAL
	 * VCOM_VAL = 0
	 * DDR_MODE = DISABLED
	 * LVDS_MODE_CE = DISABLED
	 * LVDS_MODE = DISABLED
	 * DUAL_SCAN = DISABLED
	 * E70Q10 hwcfg DisplayBusWidth=16Bits (Netronix enum value 1):
	 * SDDO_WIDTH = 16bit, PIXELS_PER_SDCLK = 8.
	 */
	reg_val =
	    ((epdc_mode->vscan_holdoff << EPDC_TCE_CTRL_VSCAN_HOLDOFF_OFFSET) &
	     EPDC_TCE_CTRL_VSCAN_HOLDOFF_MASK)
	    | EPDC_TCE_CTRL_PIXELS_PER_SDCLK_8
	    | EPDC_TCE_CTRL_SDDO_WIDTH_16BIT;
	__raw_writel(reg_val, EPDC_TCE_CTRL);

	/* EPDC_TCE_HSCAN */
	epdc_set_horizontal_timing(epdc_mode->vmode->left_margin,
				   epdc_mode->vmode->right_margin,
				   epdc_mode->vmode->hsync_len,
				   epdc_mode->vmode->hsync_len);

	/* EPDC_TCE_VSCAN */
	epdc_set_vertical_timing(epdc_mode->vmode->upper_margin,
			 epdc_mode->vmode->lower_margin,
			 epdc_mode->vmode->vsync_len);

	/* EPDC_TCE_OE */
	reg_val =
	    ((epdc_mode->sdoed_width << EPDC_TCE_OE_SDOED_WIDTH_OFFSET) &
	     EPDC_TCE_OE_SDOED_WIDTH_MASK)
	    | ((epdc_mode->sdoed_delay << EPDC_TCE_OE_SDOED_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOED_DLY_MASK)
	    | ((epdc_mode->sdoez_width << EPDC_TCE_OE_SDOEZ_WIDTH_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_WIDTH_MASK)
	    | ((epdc_mode->sdoez_delay << EPDC_TCE_OE_SDOEZ_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_DLY_MASK);
	__raw_writel(reg_val, EPDC_TCE_OE);

	/* EPDC_TCE_TIMING1 */
	__raw_writel(0x0, EPDC_TCE_TIMING1);

	/* EPDC_TCE_TIMING2 */
	reg_val =
	    ((epdc_mode->gdclk_hp_offs << EPDC_TCE_TIMING2_GDCLK_HP_OFFSET) &
	     EPDC_TCE_TIMING2_GDCLK_HP_MASK)
	    | ((epdc_mode->gdsp_offs << EPDC_TCE_TIMING2_GDSP_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING2_GDSP_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING2);

	/* EPDC_TCE_TIMING3 */
	reg_val =
	    ((epdc_mode->gdoe_offs << EPDC_TCE_TIMING3_GDOE_OFFSET_OFFSET) &
	     EPDC_TCE_TIMING3_GDOE_OFFSET_MASK)
	    | ((epdc_mode->gdclk_offs << EPDC_TCE_TIMING3_GDCLK_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING3_GDCLK_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING3);

	/*
	 * EPDC_TCE_SDCFG
	 * SDCLK_HOLD = 1
	 * SDSHR = 1
	 * NUM_CE = 1
	 * SDDO_REFORMAT = FLIP_PIXELS
	 * SDDO_INVERT = DISABLED
	 * PIXELS_PER_CE = display horizontal resolution
	 */
	num_ce = epdc_mode->num_ce;
	if (num_ce == 0)
		num_ce = 1;
	reg_val = EPDC_TCE_SDCFG_SDCLK_HOLD | EPDC_TCE_SDCFG_SDSHR
	    | ((num_ce << EPDC_TCE_SDCFG_NUM_CE_OFFSET) &
	       EPDC_TCE_SDCFG_NUM_CE_MASK)
	    | EPDC_TCE_SDCFG_SDDO_REFORMAT_FLIP_PIXELS
	    | ((epdc_mode->vmode->xres/num_ce << EPDC_TCE_SDCFG_PIXELS_PER_CE_OFFSET) &
	       EPDC_TCE_SDCFG_PIXELS_PER_CE_MASK);
	__raw_writel(reg_val, EPDC_TCE_SDCFG);

	/*
	 * EPDC_TCE_GDCFG
	 * GDRL = 1
	 * GDOE_MODE = 0;
	 * GDSP_MODE = 0;
	 */
	reg_val = EPDC_TCE_SDCFG_GDRL;
	__raw_writel(reg_val, EPDC_TCE_GDCFG);

	/*
	 * EPDC_TCE_POLARITY
	 * SDCE_POL = ACTIVE LOW
	 * SDLE_POL = ACTIVE HIGH
	 * SDOE_POL = ACTIVE HIGH
	 * GDOE_POL = ACTIVE HIGH
	 * GDSP_POL = ACTIVE LOW
	 */
	reg_val = EPDC_TCE_POLARITY_SDLE_POL_ACTIVE_HIGH
	    | EPDC_TCE_POLARITY_SDOE_POL_ACTIVE_HIGH
	    | EPDC_TCE_POLARITY_GDOE_POL_ACTIVE_HIGH;
	__raw_writel(reg_val, EPDC_TCE_POLARITY);

	/* EPDC_IRQ_MASK */
	__raw_writel(EPDC_IRQ_TCE_UNDERRUN_IRQ, EPDC_IRQ_MASK);

	/*
	 * EPDC_GPIO
	 * PWRCOM = ?
	 * PWRCTRL = ?
	 * BDR = ?
	 */
	reg_val = ((0 << EPDC_GPIO_PWRCTRL_OFFSET) & EPDC_GPIO_PWRCTRL_MASK)
	    | ((0 << EPDC_GPIO_BDR_OFFSET) & EPDC_GPIO_BDR_MASK);
	__raw_writel(reg_val, EPDC_GPIO);

	__raw_writel(fb_data->waveform_buffer_phys, EPDC_WVADDR);
	__raw_writel(fb_data->working_buffer_phys, EPDC_WB_ADDR);
	__raw_writel(fb_data->working_buffer_phys, EPDC_WB_ADDR_TCE);

	if (fb_data->buf_pix_fmt == EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N) {
		__raw_writel(EPDC_HIST1_P5N_PARAM, EPDC_HIST1_PARAM);
		__raw_writel(EPDC_HIST2_P5N_PARAM, EPDC_HIST2_PARAM);
		__raw_writel(EPDC_HIST4_P5N_PARAM, EPDC_HIST4_PARAM);
		__raw_writel(EPDC_HIST8_P5N_PARAM0, EPDC_HIST8_PARAM0);
		__raw_writel(EPDC_HIST8_P5N_PARAM1, EPDC_HIST8_PARAM1);
		__raw_writel(EPDC_HIST16_P5N_PARAM0, EPDC_HIST16_PARAM0);
		__raw_writel(EPDC_HIST16_P5N_PARAM1, EPDC_HIST16_PARAM1);
		__raw_writel(EPDC_HIST16_P5N_PARAM2, EPDC_HIST16_PARAM2);
		__raw_writel(EPDC_HIST16_P5N_PARAM3, EPDC_HIST16_PARAM3);
	} else {
		__raw_writel(EPDC_HIST1_P4N_PARAM, EPDC_HIST1_PARAM);
		__raw_writel(EPDC_HIST2_P4N_PARAM, EPDC_HIST2_PARAM);
		__raw_writel(EPDC_HIST4_P4N_PARAM, EPDC_HIST4_PARAM);
		__raw_writel(EPDC_HIST8_P4N_PARAM0, EPDC_HIST8_PARAM0);
		__raw_writel(EPDC_HIST8_P4N_PARAM1, EPDC_HIST8_PARAM1);
		__raw_writel(EPDC_HIST16_P4N_PARAM0, EPDC_HIST16_PARAM0);
		__raw_writel(EPDC_HIST16_P4N_PARAM1, EPDC_HIST16_PARAM1);
		__raw_writel(EPDC_HIST16_P4N_PARAM2, EPDC_HIST16_PARAM2);
		__raw_writel(EPDC_HIST16_P4N_PARAM3, EPDC_HIST16_PARAM3);
	}
}

static int epdc_init_settings_runtime(struct mxc_epdc_fb_data *fb_data)
{
	unsigned long timeout;
	int ret;

	/* Runtime reset callers use checked clocks and bounded reset polling. */
	ret = clk_prepare_enable(fb_data->epdc_clk_axi);
	if (ret)
		return ret;
	ret = clk_prepare_enable(fb_data->epdc_clk_pix);
	if (ret) {
		clk_disable_unprepare(fb_data->epdc_clk_axi);
		return ret;
	}

	/* Reset */
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_SET);
	timeout = jiffies + msecs_to_jiffies(100);
	while (!(__raw_readl(EPDC_CTRL) & EPDC_CTRL_CLKGATE)) {
		if (time_after_eq(jiffies, timeout)) {
			dev_err(fb_data->dev,
				"EPDC reset did not assert CLKGATE\n");
			ret = -ETIMEDOUT;
			goto out_disable_clocks;
		}
		udelay(1);
	}
	udelay(200);

	/* Netronix restart ordering avoids a stale TCE-underrun crossing from
	 * the system clock domain into the pixel clock domain.
	 */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);
	udelay(100);
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_CLEAR);
	timeout = jiffies + msecs_to_jiffies(100);
	while (__raw_readl(EPDC_CTRL) &
	       (EPDC_CTRL_SFTRST | EPDC_CTRL_CLKGATE)) {
		if (time_after_eq(jiffies, timeout)) {
			dev_err(fb_data->dev,
				"EPDC reset did not clear SFTRST/CLKGATE\n");
			ret = -ETIMEDOUT;
			goto out_disable_clocks;
		}
		udelay(1);
	}

	epdc_program_settings(fb_data);
	ret = 0;

out_disable_clocks:
	clk_disable_unprepare(fb_data->epdc_clk_axi);
	clk_disable_unprepare(fb_data->epdc_clk_pix);
	return ret;
}

static void epdc_init_settings(struct mxc_epdc_fb_data *fb_data)
{
	/* Match the original boot-time EPDC reset polling and clock handling. */
	clk_prepare_enable(fb_data->epdc_clk_axi);
	clk_prepare_enable(fb_data->epdc_clk_pix);

	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_SET);
	while (!(__raw_readl(EPDC_CTRL) & EPDC_CTRL_CLKGATE))
		;
	udelay(200);

	/* Netronix restart ordering avoids stale TCE-underrun crossing. */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);
	udelay(100);
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_CLEAR);
	while (__raw_readl(EPDC_CTRL) &
	       (EPDC_CTRL_SFTRST | EPDC_CTRL_CLKGATE))
		;

	epdc_program_settings(fb_data);
	clk_disable_unprepare(fb_data->epdc_clk_axi);
	clk_disable_unprepare(fb_data->epdc_clk_pix);
}

/* power_mutex must be held by all three helpers below. */
static int epdc_powerup_clocks_locked(struct mxc_epdc_fb_data *fb_data)
{
	int ret;

	if (!fb_data->v3p3_enabled) {
		ret = regulator_enable(fb_data->v3p3_regulator);
		if (ret)
			return ret;
		fb_data->v3p3_enabled = true;
		msleep(1);
	}

	if (!fb_data->runtime_pm_held) {
		ret = pm_runtime_get_sync(fb_data->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(fb_data->dev);
			goto err_v3p3;
		}
		fb_data->runtime_pm_held = true;
	}

	if (!fb_data->epdc_clocks_enabled) {
		ret = clk_prepare_enable(fb_data->epdc_clk_axi);
		if (ret)
			goto err_runtime;
		ret = clk_prepare_enable(fb_data->epdc_clk_pix);
		if (ret) {
			clk_disable_unprepare(fb_data->epdc_clk_axi);
			goto err_runtime;
		}
		fb_data->epdc_clocks_enabled = true;
	}

	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);
	return 0;

err_runtime:
	if (fb_data->runtime_pm_held) {
		pm_runtime_put_sync_suspend(fb_data->dev);
		fb_data->runtime_pm_held = false;
	}
err_v3p3:
	if (fb_data->v3p3_enabled) {
		regulator_disable(fb_data->v3p3_regulator);
		fb_data->v3p3_enabled = false;
	}
	return ret;
}

static int epdc_panel_powerup_locked(struct mxc_epdc_fb_data *fb_data)
{
	bool enabled_display = false;
	int ret = 0;

	if (!fb_data->display_enabled) {
		ret = regulator_enable(fb_data->display_regulator);
		if (ret)
			goto out;
		fb_data->display_enabled = true;
		enabled_display = true;
	}
	if (!fb_data->vcom_enabled) {
		ret = regulator_enable(fb_data->vcom_regulator);
		if (ret && enabled_display) {
			regulator_disable(fb_data->display_regulator);
			fb_data->display_enabled = false;
		}
		if (ret)
			goto out;
		fb_data->vcom_enabled = true;
	}
out:
	/* DISPLAY regulator enable includes the TPS65185 PWRGOOD wait result. */
	fb_data->last_panel_powerup_result = ret;
	return ret;
}

static int epdc_panel_powerdown_locked(struct mxc_epdc_fb_data *fb_data)
{
	int ret = 0;
	int rail_ret;

	if (fb_data->vcom_enabled) {
		rail_ret = regulator_disable(fb_data->vcom_regulator);
		if (!rail_ret)
			fb_data->vcom_enabled = false;
		else
			ret = rail_ret;
	}
	if (fb_data->display_enabled) {
		rail_ret = regulator_disable(fb_data->display_regulator);
		if (!rail_ret)
			fb_data->display_enabled = false;
		else if (!ret)
			ret = rail_ret;
	}
	return ret;
}

static void epdc_powerdown_clocks_locked(struct mxc_epdc_fb_data *fb_data)
{
	if (fb_data->epdc_clocks_enabled) {
		__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_SET);
		clk_disable_unprepare(fb_data->epdc_clk_pix);
		clk_disable_unprepare(fb_data->epdc_clk_axi);
		fb_data->epdc_clocks_enabled = false;
	}
	if (fb_data->runtime_pm_held) {
		pm_runtime_put_sync_suspend(fb_data->dev);
		fb_data->runtime_pm_held = false;
	}
	if (fb_data->v3p3_enabled) {
		regulator_disable(fb_data->v3p3_regulator);
		fb_data->v3p3_enabled = false;
	}
}

static void epdc_powerup(struct mxc_epdc_fb_data *fb_data)
{
	int ret;

	mutex_lock(&fb_data->power_mutex);
	if (fb_data->powering_down)
		fb_data->powering_down = false;
	fb_data->updates_active = true;
	if (fb_data->power_state == POWER_STATE_ON)
		goto out;

	dev_dbg(fb_data->dev, "EPDC Powerup\n");
	ret = epdc_powerup_clocks_locked(fb_data);
	if (ret)
		goto fail;
	ret = epdc_panel_powerup_locked(fb_data);
	if (ret) {
		epdc_powerdown_clocks_locked(fb_data);
		goto fail;
	}
	fb_data->power_state = POWER_STATE_ON;
	goto out;

fail:
	fb_data->updates_active = false;
	dev_err(fb_data->dev, "EPDC power-up failed: %d\n", ret);
out:
	mutex_unlock(&fb_data->power_mutex);
}

static void epdc_powerdown_locked(struct mxc_epdc_fb_data *fb_data)
{
	/* Caller holds power_mutex.  Runtime callers also hold queue_mutex.  The
	 * legacy boot caller deliberately skips queue/HW-idle bookkeeping.
	 */
	if (!fb_data->powering_down ||
	    fb_data->power_state == POWER_STATE_OFF)
		return;

	dev_dbg(fb_data->dev, "EPDC Powerdown\n");

	/* Disable power to the EPD panel */
	epdc_panel_powerdown_locked(fb_data);
	epdc_powerdown_clocks_locked(fb_data);

	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;

	if (fb_data->wait_for_powerdown) {
		fb_data->wait_for_powerdown = false;
		complete(&fb_data->powerdown_compl);
	}

	}

/* Boot Mode0 uses the original powerdown semantics and must not depend on
 * runtime update bookkeeping or PxP state.
 */
static void epdc_powerdown(struct mxc_epdc_fb_data *fb_data)
{
	mutex_lock(&fb_data->power_mutex);
	epdc_powerdown_locked(fb_data);
	mutex_unlock(&fb_data->power_mutex);
}

static int epdc_runtime_powerdown(struct mxc_epdc_fb_data *fb_data)
{
	int ret;

	ret = epdc_wait_for_hw_idle(fb_data, true, 5000, NULL);
	mutex_lock(&fb_data->queue_mutex);
	mutex_lock(&fb_data->power_mutex);
	if (ret) {
		if (fb_data->power_state == POWER_STATE_ON)
			fb_data->powering_down = false;
		if (fb_data->wait_for_powerdown) {
			fb_data->wait_for_powerdown = false;
			complete(&fb_data->powerdown_compl);
		}
		dev_warn_ratelimited(fb_data->dev,
			"EPDC runtime powerdown refused: hardware not idle (%d)\n",
			ret);
		goto out;
	}

	/* Recheck under the same locks held through panel and clock shutdown. */
	if (!fb_data->powering_down ||
	    fb_data->power_state == POWER_STATE_OFF) {
		ret = 0;
		goto out;
	}
	ret = epdc_hw_idle_locked(fb_data, true);
	if (ret) {
		fb_data->powering_down = false;
		if (fb_data->wait_for_powerdown) {
			fb_data->wait_for_powerdown = false;
			complete(&fb_data->powerdown_compl);
		}
		dev_warn_ratelimited(fb_data->dev,
			"EPDC runtime powerdown cancelled by final idle check (%d)\n",
			ret);
		goto out;
	}

	epdc_powerdown_locked(fb_data);
	ret = 0;
out:
	mutex_unlock(&fb_data->power_mutex);
	mutex_unlock(&fb_data->queue_mutex);
	return ret;
}

static unsigned int epdc_recovery_count_list(struct list_head *head)
{
	struct list_head *pos;
	unsigned int count = 0;

	list_for_each(pos, head)
		count++;
	return count;
}

/* Fault-only register/software snapshot, called after worker quiescence. */
static void epdc_recovery_dump(struct mxc_epdc_fb_data *fb_data,
			      const char *reason)
{
	u64 active_luts;

	mutex_lock(&fb_data->queue_mutex);
	mutex_lock(&fb_data->power_mutex);
	if (fb_data->power_state != POWER_STATE_ON ||
	    !fb_data->epdc_clocks_enabled) {
		dev_err_ratelimited(fb_data->dev,
			"EPDC %s: register dump skipped; block/clock is off state=%d clocks=%u active=%u pending=%u queued=%u collision=%u cur=%u\n",
			reason, fb_data->power_state,
			fb_data->epdc_clocks_enabled, fb_data->updates_active,
			epdc_recovery_count_list(&fb_data->upd_pending_list),
			epdc_recovery_count_list(&fb_data->upd_buf_queue),
			epdc_recovery_count_list(&fb_data->upd_buf_collision_list),
			fb_data->cur_update != NULL);
		mutex_unlock(&fb_data->power_mutex);
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}
	active_luts = (u64)__raw_readl(EPDC_STATUS_LUTS) |
		((u64)__raw_readl(EPDC_STATUS_LUTS2) << 32);
	dev_err_ratelimited(fb_data->dev,
		"EPDC %s: CTRL=%08x WV=%08x WB=%08x RES=%08x FORMAT=%08x FIFO=%08x\n",
		reason, __raw_readl(EPDC_CTRL), __raw_readl(EPDC_WVADDR),
		__raw_readl(EPDC_WB_ADDR), __raw_readl(EPDC_RES),
		__raw_readl(EPDC_FORMAT), __raw_readl(EPDC_FIFOCTRL));
	dev_err_ratelimited(fb_data->dev,
		"EPDC %s: UPD=%08x STRIDE=%08x CORD=%08x SIZE=%08x CTRL=%08x TEMP=%08x AUTOWV=%08x TCE=%08x\n",
		reason, __raw_readl(EPDC_UPD_ADDR),
		__raw_readl(EPDC_UPD_STRIDE), __raw_readl(EPDC_UPD_CORD),
		__raw_readl(EPDC_UPD_SIZE), __raw_readl(EPDC_UPD_CTRL),
		__raw_readl(EPDC_TEMP), __raw_readl(EPDC_AUTOWV_LUT),
		__raw_readl(EPDC_TCE_CTRL));
	dev_err_ratelimited(fb_data->dev,
		"EPDC %s: STATUS=%08x LUTS=%016llx IRQ=%08x MASK=%08x state=%d active=%u idle=%u down=%u pending=%u queued=%u collision=%u cur=%u pxp_lut=%x underruns=%d\n",
		reason, __raw_readl(EPDC_STATUS),
		(unsigned long long)active_luts, __raw_readl(EPDC_IRQ),
		__raw_readl(EPDC_IRQ_MASK),
		atomic_read(&fb_data->epdc_recovery_state),
		fb_data->updates_active, fb_data->waiting_for_idle,
		fb_data->powering_down,
		epdc_recovery_count_list(&fb_data->upd_pending_list),
		epdc_recovery_count_list(&fb_data->upd_buf_queue),
		epdc_recovery_count_list(&fb_data->upd_buf_collision_list),
		fb_data->cur_update != NULL,
		fb_data->pxp_conf.proc_data.lut_transform,
		atomic_read(&fb_data->tce_underrun_count));
	mutex_unlock(&fb_data->power_mutex);
	mutex_unlock(&fb_data->queue_mutex);
	if (fb_data->pxp_debug_chan)
		pxp_dump_state(fb_data->pxp_debug_chan, fb_data->dev);
}

static void epdc_complete_marker_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_marker_data *marker, int result)
{
	if (!list_empty(&marker->upd_list))
		list_del_init(&marker->upd_list);
	if (!list_empty(&marker->full_list))
		list_del_init(&marker->full_list);
	marker->result = result;
	if (marker->retained_sleep) {
		if (list_empty(&marker->sleep_done_list))
			list_add_tail(&marker->sleep_done_list,
				      &fb_data->sleep_marker_done_list);
		complete_all(&marker->update_completion);
	} else if (marker->waiting) {
		complete(&marker->update_completion);
	} else {
		kfree(marker);
	}
}

static void epdc_recovery_fail_marker_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_marker_data *marker)
{
	epdc_complete_marker_locked(fb_data, marker, -EIO);
}

static void epdc_recovery_fail_all_markers_locked(
		struct mxc_epdc_fb_data *fb_data)
{
	struct update_marker_data *marker, *tmp;

	list_for_each_entry_safe(marker, tmp, &fb_data->full_marker_list,
				 full_list)
		epdc_recovery_fail_marker_locked(fb_data, marker);
}

static void epdc_recovery_fail_desc_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *desc, bool *had_alt_buffer)
{
	struct update_marker_data *marker, *tmp;

	if (!desc)
		return;
	if (desc->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)
		*had_alt_buffer = true;
	list_for_each_entry_safe(marker, tmp, &desc->upd_marker_list, upd_list)
		epdc_recovery_fail_marker_locked(fb_data, marker);
	if (epdc_is_full_panel_gc16_cleanup(fb_data, &desc->upd_data)) {
		fb_data->cleanup_fence_active = false;
		fb_data->cleanup_fence_lut = INVALID_LUT;
	}
	epdc_free_update_desc_locked(fb_data, desc);
}

static void epdc_recovery_fail_update_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_data_list *update, bool *had_alt_buffer)
{
	if (!update)
		return;
	epdc_recovery_fail_desc_locked(fb_data, update->update_desc,
					 had_alt_buffer);
	update->update_desc = NULL;
	list_del_init(&update->list);
	list_add_tail(&update->list, &fb_data->upd_buf_free_list);
}

static bool epdc_recovery_drop_queued_state_locked(
		struct mxc_epdc_fb_data *fb_data)
{
	struct update_desc_list *desc, *tmp_desc;
	struct update_data_list *update, *tmp_update;
	struct update_marker_data *marker, *tmp_marker;
	bool had_alt_buffer = false;

	list_for_each_entry_safe(desc, tmp_desc, &fb_data->upd_pending_list,
				 list) {
		list_del_init(&desc->list);
		epdc_recovery_fail_desc_locked(fb_data, desc, &had_alt_buffer);
	}
	if (fb_data->cur_update) {
		update = fb_data->cur_update;
		fb_data->cur_update = NULL;
		epdc_recovery_fail_update_locked(fb_data, update,
						 &had_alt_buffer);
	}
	list_for_each_entry_safe(update, tmp_update, &fb_data->upd_buf_queue,
				 list)
		epdc_recovery_fail_update_locked(fb_data, update,
						 &had_alt_buffer);
	list_for_each_entry_safe(update, tmp_update,
				 &fb_data->upd_buf_collision_list, list)
		epdc_recovery_fail_update_locked(fb_data, update,
						 &had_alt_buffer);
	list_for_each_entry_safe(marker, tmp_marker, &fb_data->full_marker_list,
				 full_list)
		epdc_recovery_fail_marker_locked(fb_data, marker);

	memset(fb_data->lut_update_order, 0,
	       fb_data->num_luts * sizeof(*fb_data->lut_update_order));
	fb_data->epdc_colliding_luts = 0;
	fb_data->luts_complete_wb = 0;
	fb_data->cleanup_fence_active = false;
	fb_data->cleanup_fence_lut = INVALID_LUT;
	fb_data->updates_active = false;
	fb_data->waiting_for_wb = false;
	fb_data->waiting_for_lut = false;
	fb_data->waiting_for_lut15 = false;
	if (fb_data->waiting_for_idle) {
		fb_data->waiting_for_idle = false;
		complete_all(&fb_data->updates_done);
	}
	return had_alt_buffer;
}

static bool epdc_request_recovery(struct mxc_epdc_fb_data *fb_data)
{
	unsigned long flags;
	bool schedule = false;

	spin_lock_irqsave(&fb_data->epdc_recovery_lock, flags);
	if (!atomic_read(&fb_data->epdc_recovery_state)) {
		reinit_completion(&fb_data->epdc_recovery_done);
		fb_data->epdc_recovery_result = -EINPROGRESS;
		atomic_set(&fb_data->epdc_recovery_state, 1);
		schedule = true;
	}
	spin_unlock_irqrestore(&fb_data->epdc_recovery_lock, flags);
	if (schedule)
		schedule_work(&fb_data->epdc_recovery_work);
	return schedule;
}

static void epdc_finish_recovery(struct mxc_epdc_fb_data *fb_data, int ret)
{
	unsigned long flags;

	spin_lock_irqsave(&fb_data->epdc_recovery_lock, flags);
	fb_data->epdc_recovery_result = ret;
	atomic_set(&fb_data->epdc_recovery_state, ret ? 2 : 0);
	complete_all(&fb_data->epdc_recovery_done);
	spin_unlock_irqrestore(&fb_data->epdc_recovery_lock, flags);
}

static void epdc_recovery_work_func(struct work_struct *work)
{
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_recovery_work);
	struct mxcfb_update_data update;
	unsigned long timeout;
	int tce_count;
	int ret = 0;
	int i;
	bool irq_disabled = false;
	bool had_alt_buffer;

	tce_count = atomic_read(&fb_data->tce_underrun_count);
	dev_err_ratelimited(fb_data->dev,
		"EPDC recovery started (underruns=%d)\n", tce_count);
	disable_irq(fb_data->epdc_irq);
	irq_disabled = true;
	cancel_work_sync(&fb_data->epdc_intr_work);

	/* Wake only the resource waits that the submit worker is actually in. */
	mutex_lock(&fb_data->queue_mutex);
	if (fb_data->waiting_for_wb) {
		fb_data->waiting_for_wb = false;
		complete_all(&fb_data->update_res_free);
	}
	if (fb_data->waiting_for_lut) {
		fb_data->waiting_for_lut = false;
		complete_all(&fb_data->update_res_free);
	}
	if (fb_data->waiting_for_lut15) {
		fb_data->waiting_for_lut15 = false;
		complete_all(&fb_data->lut15_free);
	}
	mutex_unlock(&fb_data->queue_mutex);

	cancel_work(&fb_data->epdc_submit_work);
	timeout = jiffies + msecs_to_jiffies(5000);
	for (i = 0; i < 500; i++) {
		cancel_work(&fb_data->epdc_submit_work);
		if (!(work_busy(&fb_data->epdc_submit_work) & WORK_BUSY_RUNNING))
			break;
		if (time_after_eq(jiffies, timeout))
			break;
		msleep(10);
	}
	if (work_busy(&fb_data->epdc_submit_work) & WORK_BUSY_RUNNING) {
		ret = -ETIMEDOUT;
		goto fail;
	}
	cancel_work(&fb_data->epdc_submit_work);
	epdc_recovery_dump(fb_data, "before-recovery");

	cancel_delayed_work_sync(&fb_data->epdc_done_work);
	mutex_lock(&fb_data->power_mutex);
	fb_data->powering_down = false;
	mutex_unlock(&fb_data->power_mutex);

	mutex_lock(&fb_data->queue_mutex);
	had_alt_buffer = epdc_recovery_drop_queued_state_locked(fb_data);
	mutex_unlock(&fb_data->queue_mutex);

	ret = epdc_init_settings_runtime(fb_data);
	if (ret)
		goto fail;

	enable_irq(fb_data->epdc_irq);
	irq_disabled = false;

	if (had_alt_buffer) {
		dev_err(fb_data->dev,
			"EPDC recovery cannot reconstruct an outstanding alternate-buffer update\n");
		ret = -EIO;
		goto fail;
	}

	if (fb_data->blank == FB_BLANK_UNBLANK) {
		memset(&update, 0, sizeof(update));
		update.update_region.left = 0;
		update.update_region.top = 0;
		update.update_region.width = fb_data->epdc_fb_var.xres;
		update.update_region.height = fb_data->epdc_fb_var.yres;
		update.update_mode = UPDATE_MODE_FULL;
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
		update.temp = TEMP_USE_AMBIENT;

		mutex_lock(&fb_data->queue_mutex);
		fb_data->recovery_refresh_active = true;
		mutex_unlock(&fb_data->queue_mutex);
		ret = __mxc_epdc_fb_send_single_update(&update, &fb_data->info,
						      true, NULL, 0);
		if (!ret)
			ret = mxc_epdc_fb_flush_updates(fb_data);
		mutex_lock(&fb_data->queue_mutex);
		fb_data->recovery_refresh_active = false;
		if (!ret && (!list_empty(&fb_data->upd_pending_list) ||
			     !list_empty(&fb_data->upd_buf_queue) ||
			     !list_empty(&fb_data->upd_buf_collision_list) ||
			     fb_data->cur_update || fb_data->updates_active ||
			     epdc_recovery_count_list(&fb_data->upd_buf_free_list) !=
				fb_data->max_num_updates))
			ret = -EIO;
		mutex_unlock(&fb_data->queue_mutex);
		if (ret)
			goto fail;
		if (atomic_read(&fb_data->tce_underrun_count) != tce_count) {
			ret = -EIO;
			goto fail;
		}
	}

	dev_info(fb_data->dev, "EPDC recovery completed\n");
	epdc_finish_recovery(fb_data, 0);
	return;

fail:
	if (irq_disabled)
		enable_irq(fb_data->epdc_irq);
	mutex_lock(&fb_data->queue_mutex);
	fb_data->recovery_refresh_active = false;
	epdc_recovery_fail_all_markers_locked(fb_data);
	mutex_unlock(&fb_data->queue_mutex);
	dev_err(fb_data->dev,
		"EPDC recovery failed (%d); updates remain blocked\n", ret);
	epdc_recovery_dump(fb_data, "recovery-failed");
	epdc_finish_recovery(fb_data, ret ? ret : -EIO);
}

static void epdc_init_sequence(struct mxc_epdc_fb_data *fb_data)
{
	/* Initialize EPDC, passing pointer to EPDC registers */
	epdc_init_settings(fb_data);
	fb_data->in_init = true;
	epdc_powerup(fb_data);
	draw_mode0(fb_data);
	/* Force power down event */
	fb_data->powering_down = true;
	epdc_powerdown(fb_data);
	fb_data->updates_active = false;
}

static int mxc_epdc_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else
		return -EINVAL;

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxc_epdc_fb_setcolreg(u_int regno, u_int red, u_int green,
				 u_int blue, u_int transp, struct fb_info *info)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val = _chan_to_field(red, &info->var.red);
			val |= _chan_to_field(green, &info->var.green);
			val |= _chan_to_field(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static int mxc_epdc_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int i;

	dev_dbg(fb_data->dev, "setcmap\n");

	if (info->fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR) {
		/* Only support an 8-bit, 256 entry lookup */
		if (cmap->len != 256)
			return 1;

		mxc_epdc_fb_flush_updates(fb_data);

		mutex_lock(&fb_data->pxp_mutex);
		/*
		 * Store colormap in pxp_conf structure for later transmit
		 * to PxP during update process to convert gray pixels.
		 *
		 * Since red=blue=green for pseudocolor visuals, we can
		 * just use red values.
		 */
		for (i = 0; i < 256; i++)
			fb_data->pxp_conf.proc_data.lut_map[i] = cmap->red[i] & 0xFF;

		fb_data->pxp_conf.proc_data.lut_map_updated = true;

		mutex_unlock(&fb_data->pxp_mutex);
	} else {
		red     = cmap->red;
		green   = cmap->green;
		blue    = cmap->blue;
		transp  = cmap->transp;
		index   = cmap->start;

		for (count = 0; count < cmap->len; count++) {
			if (transp)
				trans = *transp++;
			r = mxc_epdc_fb_setcolreg(index++, *red++, *green++, *blue++,
						trans, info);
			if (r != 0)
				return r;
		}
	}

	return 0;
}

static void adjust_coordinates(u32 xres, u32 yres, u32 rotation,
	struct mxcfb_rect *update_region, struct mxcfb_rect *adj_update_region)
{
	u32 temp;

	/* If adj_update_region == NULL, pass result back in update_region */
	/* If adj_update_region == valid, use it to pass back result */
	if (adj_update_region)
		switch (rotation) {
		case FB_ROTATE_UR:
			adj_update_region->top = update_region->top;
			adj_update_region->left = update_region->left;
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			break;
		case FB_ROTATE_CW:
			adj_update_region->top = update_region->left;
			adj_update_region->left = yres -
				(update_region->top + update_region->height);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		case FB_ROTATE_UD:
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			adj_update_region->top = yres -
				(update_region->top + update_region->height);
			adj_update_region->left = xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			adj_update_region->left = update_region->top;
			adj_update_region->top = xres -
				(update_region->left + update_region->width);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		}
	else
		switch (rotation) {
		case FB_ROTATE_UR:
			/* No adjustment needed */
			break;
		case FB_ROTATE_CW:
			temp = update_region->top;
			update_region->top = update_region->left;
			update_region->left = yres -
				(temp + update_region->height);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		case FB_ROTATE_UD:
			update_region->top = yres -
				(update_region->top + update_region->height);
			update_region->left = xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			temp = update_region->left;
			update_region->left = update_region->top;
			update_region->top = xres -
				(temp + update_region->width);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		}
}

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mxc_epdc_fb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	if (var->grayscale)
		fix->visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	else
		fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}

/*
 * This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 *
 */
static int mxc_epdc_fb_set_par(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	struct fb_var_screeninfo *screeninfo = &fb_data->info.var;
	struct imx_epdc_fb_mode *epdc_modes = fb_data->pdata->epdc_mode;
	int i;
	int ret;
	__u32 xoffset_old, yoffset_old;

	/*
	 * Can't change the FB parameters until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_epdc_fb_flush_updates(fb_data);

	mutex_lock(&fb_data->queue_mutex);
	/*
	 * Set all screeninfo except for xoffset/yoffset
	 * Subsequent call to pan_display will handle those.
	 */
	xoffset_old = fb_data->epdc_fb_var.xoffset;
	yoffset_old = fb_data->epdc_fb_var.yoffset;
	fb_data->epdc_fb_var = *screeninfo;
	fb_data->epdc_fb_var.xoffset = xoffset_old;
	fb_data->epdc_fb_var.yoffset = yoffset_old;
	mutex_unlock(&fb_data->queue_mutex);

	mutex_lock(&fb_data->pxp_mutex);

	/*
	 * Update PxP config data (used to process FB regions for updates)
	 * based on FB info and processing tasks required
	 */

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = screeninfo->xres;
	proc_data->drect.height = proc_data->srect.height = screeninfo->yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = screeninfo->rotate;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * configure S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	if (screeninfo->grayscale)
		pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_GREY;
	else {
		switch (screeninfo->bits_per_pixel) {
		case 16:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		case 24:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB24;
			break;
		case 32:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_XRGB32;
			break;
		default:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		}
	}
	pxp_conf->s0_param.width = screeninfo->xres_virtual;
	pxp_conf->s0_param.height = screeninfo->yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = screeninfo->xres;
	pxp_conf->out_param.height = screeninfo->yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	mutex_unlock(&fb_data->pxp_mutex);

	/*
	 * If HW not yet initialized, check to see if we are being sent
	 * an initialization request.
	 */
	if (!fb_data->hw_ready) {
		struct fb_videomode mode;
		u32 xres_temp;

		fb_var_to_videomode(&mode, screeninfo);

		/* When comparing requested fb mode,
		   we need to use unrotated dimensions */
		if ((screeninfo->rotate == FB_ROTATE_CW) ||
			(screeninfo->rotate == FB_ROTATE_CCW)) {
			xres_temp = mode.xres;
			mode.xres = mode.yres;
			mode.yres = xres_temp;
		}

		/*
		* If requested video mode does not match current video
		* mode, search for a matching panel.
		*/
		if (fb_data->cur_mode &&
			!fb_mode_is_equal(fb_data->cur_mode->vmode,
			&mode)) {
			bool found_match = false;

			/* Match videomode against epdc modes */
			for (i = 0; i < fb_data->pdata->num_modes; i++) {
				if (!fb_mode_is_equal(epdc_modes[i].vmode,
					&mode))
					continue;
				fb_data->cur_mode = &epdc_modes[i];
				found_match = true;
				break;
			}

			if (!found_match) {
				dev_err(fb_data->dev,
					"Failed to match requested "
					"video mode\n");
				return EINVAL;
			}
		}

		/* Found a match - Grab timing params */
		screeninfo->left_margin = mode.left_margin;
		screeninfo->right_margin = mode.right_margin;
		screeninfo->upper_margin = mode.upper_margin;
		screeninfo->lower_margin = mode.lower_margin;
		screeninfo->hsync_len = mode.hsync_len;
		screeninfo->vsync_len = mode.vsync_len;

		fb_data->hw_initializing = true;

		/* Initialize EPDC settings and init panel */
		ret =
		    mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
		if (ret) {
			dev_err(fb_data->dev,
				"Failed to load panel waveform data\n");
			return ret;
		}
	}

	/*
	 * EOF sync delay (in us) should be equal to the vscan holdoff time
	 * VSCAN_HOLDOFF time = (VSCAN_HOLDOFF value + 1) * Vertical lines
	 * Add 25us for additional margin
	 */
	fb_data->eof_sync_period = (fb_data->cur_mode->vscan_holdoff + 1) *
		1000000/(fb_data->cur_mode->vmode->refresh *
		(fb_data->cur_mode->vmode->upper_margin +
		fb_data->cur_mode->vmode->yres +
		fb_data->cur_mode->vmode->lower_margin +
		fb_data->cur_mode->vmode->vsync_len)) + 25;

	mxc_epdc_fb_set_fix(info);

	return 0;
}

static int mxc_epdc_fb_check_var(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 8))
		var->bits_per_pixel = default_bpp;

	switch (var->bits_per_pixel) {
	case 8:
		if (var->grayscale != 0) {
			/*
			 * For 8-bit grayscale, R, G, and B offset are equal.
			 *
			 */
			var->red.length = 8;
			var->red.offset = 0;
			var->red.msb_right = 0;

			var->green.length = 8;
			var->green.offset = 0;
			var->green.msb_right = 0;

			var->blue.length = 8;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		} else {
			var->red.length = 3;
			var->red.offset = 5;
			var->red.msb_right = 0;

			var->green.length = 3;
			var->green.offset = 2;
			var->green.msb_right = 0;

			var->blue.length = 2;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		}
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	switch (var->rotate) {
	case FB_ROTATE_UR:
	case FB_ROTATE_UD:
		var->xres = fb_data->native_width;
		var->yres = fb_data->native_height;
		break;
	case FB_ROTATE_CW:
	case FB_ROTATE_CCW:
		var->xres = fb_data->native_height;
		var->yres = fb_data->native_width;
		break;
	default:
		/* Invalid rotation value */
		var->rotate = 0;
		dev_dbg(fb_data->dev, "Invalid rotation request\n");
		return -EINVAL;
	}

	var->xres_virtual = ALIGN(var->xres, 32);
	var->yres_virtual = ALIGN(var->yres, 128) * fb_data->num_screens;

	var->height = -1;
	var->width = -1;

	return 0;
}

void mxc_epdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,
	struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	mutex_lock(&fb_data->queue_mutex);

	memcpy(&fb_data->wv_modes, modes, sizeof(struct mxcfb_waveform_modes));

	/* Set flag to ensure that new waveform modes
	 * are programmed into EPDC before next update */
	fb_data->wv_modes_update = true;

	mutex_unlock(&fb_data->queue_mutex);
}
EXPORT_SYMBOL(mxc_epdc_fb_set_waveform_modes);

static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data, int temp)
{
	int i;
	int index = -1;

	if (fb_data->trt_entries == 0) {
		dev_err(fb_data->dev,
			"No TRT exists...using default temp index\n");
		return DEFAULT_TEMP_INDEX;
	}

	/* Search temperature ranges for a match */
	for (i = 0; i < fb_data->trt_entries - 1; i++) {
		if ((temp >= fb_data->temp_range_bounds[i])
			&& (temp < fb_data->temp_range_bounds[i+1])) {
			index = i;
			break;
		}
	}

	if (index < 0) {
		dev_err(fb_data->dev,
			"No TRT index match...using default temp index\n");
		return DEFAULT_TEMP_INDEX;
	}

	dev_dbg(fb_data->dev, "Using temperature index %d\n", index);

	return index;
}

int mxc_epdc_fb_set_temperature(int temperature, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	/* Store temp index. Used later when configuring updates. */
	mutex_lock(&fb_data->queue_mutex);
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, temperature);
	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_temperature);

int mxc_epdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	dev_dbg(fb_data->dev, "Setting auto update mode to %d\n", auto_mode);

	if ((auto_mode == AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		|| (auto_mode == AUTO_UPDATE_MODE_REGION_MODE))
		fb_data->auto_mode = auto_mode;
	else {
		dev_err(fb_data->dev, "Invalid auto update mode parameter.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_auto_update);

int mxc_epdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	int ret;

	dev_dbg(fb_data->dev, "Setting optimization level to %d\n", upd_scheme);

	/*
	 * Can't change the scheme until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	ret = mxc_epdc_fb_flush_updates(fb_data);
	if (ret)
		return ret;

	if ((upd_scheme == UPDATE_SCHEME_SNAPSHOT)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE_AND_MERGE))
		fb_data->upd_scheme = upd_scheme;
	else {
		dev_err(fb_data->dev, "Invalid update scheme specified.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_upd_scheme);

static void copy_before_process(struct mxc_epdc_fb_data *fb_data,
	struct update_data_list *upd_data_list)
{
	struct mxcfb_update_data *upd_data =
		&upd_data_list->update_desc->upd_data;
	int i;
	unsigned char *temp_buf_ptr = fb_data->virt_addr_copybuf;
	unsigned char *src_ptr;
	struct mxcfb_rect *src_upd_region;
	int temp_buf_stride;
	int src_stride;
	int bpp = fb_data->epdc_fb_var.bits_per_pixel;
	int left_offs, right_offs;
	int x_trailing_bytes, y_trailing_bytes;
	int alt_buf_offset;

	/* Set source buf pointer based on input source, panning, etc. */
	if (upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_upd_region = &upd_data->alt_buffer_data.alt_update_region;
		src_stride =
			upd_data->alt_buffer_data.width * bpp/8;
		alt_buf_offset = upd_data->alt_buffer_data.phys_addr -
			fb_data->info.fix.smem_start;
		src_ptr = fb_data->info.screen_base + alt_buf_offset
			+ src_upd_region->top * src_stride;
	} else {
		src_upd_region = &upd_data->update_region;
		src_stride = fb_data->epdc_fb_var.xres_virtual * bpp/8;
		if (upd_data_list->update_desc->source_snapshot)
			src_ptr = (u8 *)upd_data_list->update_desc->source_snapshot->virt_addr
				+ src_upd_region->top * src_stride;
		else
			src_ptr = (u8 *)fb_data->info.screen_base
				+ upd_data_list->update_desc->fb_offset
				+ src_upd_region->top * src_stride;
	}

	temp_buf_stride = ALIGN(src_upd_region->width, 8) * bpp/8;
	left_offs = src_upd_region->left * bpp/8;
	right_offs = src_upd_region->width * bpp/8;
	x_trailing_bytes = (ALIGN(src_upd_region->width, 8)
		- src_upd_region->width) * bpp/8;

	for (i = 0; i < src_upd_region->height; i++) {
		/* Copy the full line */
		memcpy(temp_buf_ptr, src_ptr + left_offs,
			src_upd_region->width * bpp/8);

		/*
		 * Pixels outside the visible 1404-wide source are still consumed by
		 * PxP's 8x8 block. White is the only safe E-Ink padding value; zero
		 * created a black block at the rotated panel edge.
		 */
		if (src_upd_region->width & 0x7) {
			memset(temp_buf_ptr + right_offs, 0xff,
				x_trailing_bytes);
		}

		temp_buf_ptr += temp_buf_stride;
		src_ptr += src_stride;
	}

	/* Clear any unwanted pixels at the bottom of the end of each line */
	if (src_upd_region->height & 0x7) {
		y_trailing_bytes = (ALIGN(src_upd_region->height, 8)
			- src_upd_region->height) *
			ALIGN(src_upd_region->width, 8) * bpp/8;
		memset(temp_buf_ptr, 0xff, y_trailing_bytes);
	}
}


static bool epdc_is_full_panel_gc16_cleanup(
				struct mxc_epdc_fb_data *fb_data,
				const struct mxcfb_update_data *upd)
{
	const struct mxcfb_rect *region;

	if (!fb_data || !upd)
		return false;

	region = &upd->update_region;
	return upd->update_mode == UPDATE_MODE_FULL &&
		upd->waveform_mode == fb_data->wv_modes.mode_gc16 &&
		region->left == 0 &&
		region->top == 0 &&
		region->width == fb_data->epdc_fb_var.xres &&
		region->height == fb_data->epdc_fb_var.yres;
}

static bool epdc_is_cleanup_desc(struct mxc_epdc_fb_data *fb_data,
						 const struct update_desc_list *desc)
{
	return epdc_is_full_panel_gc16_cleanup(fb_data,
		desc ? &desc->upd_data : NULL);
}

struct epdc_pending_coalesce_plan {
	struct update_desc_list *oldest;
	struct mxcfb_rect damage;
	u32 desc_count;
	u32 generation_count;
	u32 damage_expands;
};

static bool epdc_rect_valid_for_coalesce(struct mxc_epdc_fb_data *fb_data,
					 const struct mxcfb_rect *rect)
{
	u32 xres = fb_data->epdc_fb_var.xres;
	u32 yres = fb_data->epdc_fb_var.yres;

	return rect->width && rect->height &&
		rect->left < xres && rect->top < yres &&
		rect->width <= xres - rect->left &&
		rect->height <= yres - rect->top;
}

static bool epdc_coalesce_union_fits(struct mxc_epdc_fb_data *fb_data,
				     const struct mxcfb_rect *a,
				     const struct mxcfb_rect *b,
				     struct mxcfb_rect *combined)
{
	u32 right_a, right_b, bottom_a, bottom_b;

	if (!epdc_rect_valid_for_coalesce(fb_data, a) ||
	    !epdc_rect_valid_for_coalesce(fb_data, b))
		return false;

	right_a = a->left + a->width;
	right_b = b->left + b->width;
	bottom_a = a->top + a->height;
	bottom_b = b->top + b->height;
	combined->left = min(a->left, b->left);
	combined->top = min(a->top, b->top);
	combined->width = max(right_a, right_b) - combined->left;
	combined->height = max(bottom_a, bottom_b) - combined->top;

	if (fb_data->restrict_width) {
		u32 max_width = EPDC_V2_MAX_UPDATE_WIDTH;
		u32 combined_width = combined->width;

		if (fb_data->epdc_fb_var.rotate != FB_ROTATE_UR)
			max_width -= EPDC_V2_ROTATION_ALIGNMENT;
		if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_CW) ||
		    (fb_data->epdc_fb_var.rotate == FB_ROTATE_CCW))
			combined_width = combined->height;
		if (combined_width > max_width)
			return false;
	}

	return true;
}

static bool epdc_is_ordinary_display_update(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *desc, bool require_snapshot)
{
	if (!desc || desc->upd_data.update_marker != 0 ||
	    desc->upd_data.update_mode != UPDATE_MODE_PARTIAL ||
	    desc->upd_data.waveform_mode != WAVEFORM_MODE_AUTO ||
	    desc->upd_data.temp != TEMP_USE_AMBIENT ||
	    desc->upd_data.flags != 0 ||
	    epdc_is_cleanup_desc(fb_data, desc) ||
	    !epdc_rect_valid_for_coalesce(fb_data,
					   &desc->upd_data.update_region))
		return false;

	if (require_snapshot &&
	    (!desc->source_snapshot ||
	     desc->source_snapshot->generation != desc->fb_source_generation ||
	     desc->source_snapshot->fb_offset != desc->fb_offset))
		return false;

	return true;
}

static bool epdc_is_ordinary_ui_update(struct mxc_epdc_fb_data *fb_data,
					       struct update_desc_list *desc,
					       bool require_snapshot)
{
	return desc && !desc->collision_resubmit &&
		epdc_is_ordinary_display_update(fb_data, desc, require_snapshot);
}

static bool epdc_is_ordinary_collision_update(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *desc)
{
	return epdc_is_ordinary_display_update(fb_data, desc, false) &&
		list_empty(&desc->upd_marker_list);
}

/*
 * Collapse only a compatible suffix of upd_pending_list.  The submit worker
 * removes descriptors under the same queue_mutex, so anything no longer on
 * this list is already worker-owned and is deliberately outside this policy.
 */
static bool epdc_plan_pending_coalesce_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *newest,
		struct epdc_pending_coalesce_plan *plan)
{
	struct update_desc_list *candidate;
	u64 last_generation = 0;
	bool have_generation = false;

	memset(plan, 0, sizeof(*plan));
	if (!fb_data->source_snapshot_enabled ||
	    fb_data->upd_scheme != UPDATE_SCHEME_QUEUE_AND_MERGE)
		return false;

	if (!epdc_is_ordinary_ui_update(fb_data, newest, false)) {
		fb_data->coalesce_skipped_ineligible++;
		if (newest->upd_data.update_marker != 0)
			fb_data->coalesce_marker_skips++;
		return false;
	}

	fb_data->coalesce_passes++;
	plan->damage = newest->upd_data.update_region;
	if (list_empty(&fb_data->upd_pending_list))
		return true;

	list_for_each_entry_reverse(candidate, &fb_data->upd_pending_list, list) {
		struct mxcfb_rect combined;
		const u64 generation = candidate->fb_source_generation;

		if (candidate->upd_data.update_marker != 0) {
			fb_data->coalesce_marker_skips++;
			break;
		}
		if (!epdc_is_ordinary_ui_update(fb_data, candidate, true) ||
		    candidate->update_order >= newest->update_order ||
		    generation > newest->fb_source_generation ||
		    candidate->upd_data.update_mode != newest->upd_data.update_mode ||
		    candidate->upd_data.waveform_mode != newest->upd_data.waveform_mode ||
		    candidate->upd_data.temp != newest->upd_data.temp ||
		    candidate->upd_data.flags != newest->upd_data.flags) {
			fb_data->coalesce_skipped_ineligible++;
			break;
		}

		if (!epdc_coalesce_union_fits(fb_data, &plan->damage,
					      &candidate->upd_data.update_region,
					      &combined))
			break;

		if (combined.left != plan->damage.left ||
		    combined.top != plan->damage.top ||
		    combined.width != plan->damage.width ||
		    combined.height != plan->damage.height)
			plan->damage_expands++;
		plan->damage = combined;
		plan->oldest = candidate;
		plan->desc_count++;
		if (!have_generation || generation != last_generation) {
			plan->generation_count++;
			last_generation = generation;
			have_generation = true;
		}
	}

	return true;
}

static bool epdc_reuse_pending_snapshot_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *newest,
		const struct epdc_pending_coalesce_plan *plan)
{
	struct e70q10_source_snapshot *slot = NULL;
	struct update_desc_list *candidate;
	u32 i;

	if (!plan->desc_count || !plan->oldest ||
	    !fb_data->source_snapshots ||
	    (u64)newest->fb_offset + fb_data->source_snapshot_copy_size >
			(u64)fb_data->map_size)
		return false;

	for (i = 0; i < fb_data->source_snapshot_count; i++) {
		struct e70q10_source_snapshot *possible =
			&fb_data->source_snapshots[i];
		u32 pending_refs = 0;

		list_for_each_entry_reverse(candidate,
					    &fb_data->upd_pending_list, list) {
			if (candidate->source_snapshot == possible)
				pending_refs++;
			if (candidate == plan->oldest)
				break;
		}
		/* No worker, active, collision, or older pending descriptor may
		 * still read a slot that is overwritten with the newest frame. */
		if (pending_refs && possible->refs == pending_refs) {
			slot = possible;
			break;
		}
	}
	if (!slot)
		return false;

	memcpy(slot->virt_addr,
	       (u8 *)fb_data->info.screen_base + newest->fb_offset,
	       fb_data->source_snapshot_copy_size);
	if (fb_data->source_snapshot_size > fb_data->source_snapshot_copy_size)
		memset((u8 *)slot->virt_addr + fb_data->source_snapshot_copy_size,
		       0xff, fb_data->source_snapshot_size -
		       fb_data->source_snapshot_copy_size);
	wmb();
	slot->fb_offset = newest->fb_offset;
	slot->generation = newest->fb_source_generation;
	slot->refs++; /* new descriptor reference; pending refs are put below */
	newest->source_snapshot = slot;
	fb_data->source_snapshot_copies++;
	return true;
}

static void epdc_apply_pending_coalesce_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *newest,
		const struct epdc_pending_coalesce_plan *plan)
{
	struct update_desc_list *candidate;
	bool done;

	if (!plan->desc_count || !plan->oldest)
		return;

	newest->upd_data.update_region = plan->damage;
	for (;;) {
		candidate = list_entry(fb_data->upd_pending_list.prev,
				       struct update_desc_list, list);
		done = candidate == plan->oldest;
		list_del_init(&candidate->list);
		epdc_free_update_desc_locked(fb_data, candidate);
		fb_data->coalesce_superseded_descs++;
		if (done)
			break;
	}
	fb_data->coalesce_superseded_generations +=
		plan->generation_count;
	fb_data->coalesce_damage_expands += plan->damage_expands;
	if (plan->damage.left == 0 && plan->damage.top == 0 &&
	    plan->damage.width == fb_data->epdc_fb_var.xres &&
	    plan->damage.height == fb_data->epdc_fb_var.yres)
		fb_data->coalesce_full_bounds++;
}

/*
 * Preserve one newest ordinary framebuffer state when all normal immutable
 * snapshots are owned by worker/active/collision descriptors.  This reserve
 * is never submitted directly: the submit worker first copies it into a normal
 * free snapshot slot, so the reserve remains a bounded pending-state latch.
 */
static bool e70q10_defer_latest_update_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_desc_list *newest,
		const struct epdc_pending_coalesce_plan *plan)
{
	struct update_desc_list *deferred = fb_data->deferred_latest_desc;
	struct update_desc_list *candidate, *tmp;
	struct e70q10_source_snapshot *reserve =
		&fb_data->deferred_latest_snapshot;
	struct mxcfb_rect damage;

	if (!fb_data->deferred_latest_enabled ||
	    fb_data->upd_scheme != UPDATE_SCHEME_QUEUE_AND_MERGE ||
	    !epdc_is_ordinary_ui_update(fb_data, newest, false) ||
	    !plan || !epdc_rect_valid_for_coalesce(fb_data,
						&newest->upd_data.update_region) ||
	    (u64)newest->fb_offset + fb_data->source_snapshot_copy_size >
			(u64)fb_data->map_size)
		return false;

	damage = plan->damage;
	if (deferred) {
		if (list_empty(&fb_data->upd_pending_list) ||
		    deferred->source_snapshot != reserve ||
		    !deferred->deferred_latest || !plan->desc_count ||
		    !plan->oldest || newest->update_order <= deferred->update_order)
			return false;

		e70q10_copy_frame_to_snapshot_locked(fb_data, reserve,
			newest->fb_offset, newest->fb_source_generation);
		fb_data->source_snapshot_copies++;
		if (fb_data->upd_pending_list.prev != &deferred->list) {
			/*
			 * A marker, cleanup, or other barrier may have been queued after
			 * the old latch. Keep those descriptors in order and move the
			 * latest ordinary state behind them. Use full-screen damage because
			 * the intervening barrier may have changed any panel pixel.
			 */
			deferred->upd_data = newest->upd_data;
			deferred->upd_data.update_region.left = 0;
			deferred->upd_data.update_region.top = 0;
			deferred->upd_data.update_region.width =
				fb_data->epdc_fb_var.xres;
			deferred->upd_data.update_region.height =
				fb_data->epdc_fb_var.yres;
			deferred->fb_offset = newest->fb_offset;
			deferred->fb_source_generation =
				newest->fb_source_generation;
			deferred->fb_page_index = newest->fb_page_index;
			deferred->fb_page_generation =
				newest->fb_page_generation;
			deferred->fb_page_generation_valid =
				newest->fb_page_generation_valid;
			deferred->update_order = newest->update_order;
			reserve->fb_offset = newest->fb_offset;
			reserve->generation = newest->fb_source_generation;
			list_move_tail(&deferred->list,
				&fb_data->upd_pending_list);
			fb_data->deferred_latest_retry_replaced++;
			fb_data->latest_accepted_generation = max(
				fb_data->latest_accepted_generation,
				newest->fb_source_generation);
			e70q10_source_snapshot_put_locked(fb_data,
				newest->source_snapshot);
			newest->source_snapshot = NULL;
			kfree(newest);
			return true;
		}

		list_for_each_entry_safe_reverse(candidate, tmp,
						 &fb_data->upd_pending_list, list) {
			if (candidate == deferred) {
				if (candidate == plan->oldest)
					break;
				continue;
			}
			{
				bool done = candidate == plan->oldest;

				list_del_init(&candidate->list);
				e70q10_source_snapshot_put_locked(fb_data,
					candidate->source_snapshot);
				candidate->source_snapshot = NULL;
				kfree(candidate);
				fb_data->coalesce_superseded_descs++;
				if (done)
					break;
			}
		}

		deferred->upd_data = newest->upd_data;
		deferred->upd_data.update_region = damage;
		deferred->fb_offset = newest->fb_offset;
		deferred->fb_source_generation = newest->fb_source_generation;
		deferred->fb_page_index = newest->fb_page_index;
		deferred->fb_page_generation = newest->fb_page_generation;
		deferred->fb_page_generation_valid =
			newest->fb_page_generation_valid;
		deferred->update_order = newest->update_order;
		reserve->fb_offset = newest->fb_offset;
		reserve->generation = newest->fb_source_generation;
		fb_data->deferred_latest_retry_replaced++;
		fb_data->coalesce_superseded_generations +=
			plan->generation_count;
		fb_data->latest_accepted_generation = max(
			fb_data->latest_accepted_generation,
			newest->fb_source_generation);
		epdc_free_update_desc_locked(fb_data, newest);
		return true;
	}

	if (reserve->refs)
		return false;
	e70q10_copy_frame_to_snapshot_locked(fb_data, reserve,
		newest->fb_offset, newest->fb_source_generation);
	fb_data->source_snapshot_copies++;
	reserve->refs = 1;
	newest->source_snapshot = reserve;
	newest->deferred_latest = true;
	if (plan->desc_count)
		epdc_apply_pending_coalesce_locked(fb_data, newest, plan);
	list_add_tail(&newest->list, &fb_data->upd_pending_list);
	fb_data->deferred_latest_desc = newest;
	fb_data->deferred_latest_retry_armed++;
	fb_data->latest_accepted_generation =
		newest->fb_source_generation;
	return true;
}

static int epdc_process_update(struct update_data_list *upd_data_list,
				   struct mxc_epdc_fb_data *fb_data)
{
	struct mxcfb_rect *src_upd_region; /* Region of src buffer for update */
	struct mxcfb_rect pxp_upd_region;
	struct mxcfb_rect pxp_region_before_origin_align;
	struct mxcfb_rect process_region;
	u32 src_width, src_height;
	u32 offset_from_4, bytes_per_pixel;
	u32 post_rotation_xcoord, post_rotation_ycoord, width_pxp_blocks;
	u32 pxp_input_offs, pxp_output_offs, pxp_output_shift;
	u32 pxp_output_rows, pxp_output_bytes;
	u32 hist_stat = 0;
	int width_unaligned, height_unaligned;
	bool input_unaligned = false;
	bool line_overflow = false;
	bool collision_unaligned;
	int pix_per_line_added;
	bool use_temp_buf = false;
	struct mxcfb_rect temp_buf_upd_region;
	struct update_desc_list *upd_desc_list = upd_data_list->update_desc;

	int ret;


	/*
	 * Gotta do a whole bunch of buffer ptr manipulation to
	 * work around HW restrictions for PxP & EPDC
	 * Note: Applies to pre-2.0 versions of EPDC/PxP
	 */

	/*
	 * Are we using FB or an alternate (overlay)
	 * buffer for source of update?
	 */
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_width = upd_desc_list->upd_data.alt_buffer_data.width;
		src_height = upd_desc_list->upd_data.alt_buffer_data.height;
		src_upd_region = &upd_desc_list->upd_data.alt_buffer_data.alt_update_region;
	} else {
		src_width = fb_data->epdc_fb_var.xres_virtual;
		src_height = fb_data->epdc_fb_var.yres;
		src_upd_region = &upd_desc_list->upd_data.update_region;
	}
	process_region = *src_upd_region;

	bytes_per_pixel = fb_data->epdc_fb_var.bits_per_pixel/8;
	/*
	 * SW workaround for PxP limitation (for pre-v2.0 HW)
	 *
	 * There are 3 cases where we cannot process the update data
	 * directly from the input buffer:
	 *
	 * 1) PxP must process 8x8 pixel blocks, and all pixels in each block
	 * are considered for auto-waveform mode selection. If the
	 * update region is not 8x8 aligned, additional unwanted pixels
	 * will be considered in auto-waveform mode selection.
	 *
	 * 2) PxP input must be 32-bit aligned, so any update
	 * address not 32-bit aligned must be shifted to meet the
	 * 32-bit alignment.  The PxP will thus end up processing pixels
	 * outside of the update region to satisfy this alignment restriction,
	 * which can affect auto-waveform mode selection.
	 *
	 * 3) If input fails 32-bit alignment, and the resulting expansion
	 * of the processed region would add at least 8 pixels more per
	 * line than the original update line width, the EPDC would
	 * cause screen artifacts by incorrectly handling the 8+ pixels
	 * at the end of each line.
	 *
	 * Workaround is to copy from source buffer into a temporary
	 * buffer, which we pad with white pixels to match the 8x8 alignment
	 * requirement. This temp buffer becomes the input to the PxP.
	 */
	width_unaligned = src_upd_region->width & 0x7;
	height_unaligned = src_upd_region->height & 0x7;

	offset_from_4 = src_upd_region->left & 0x3;
	input_unaligned = ((offset_from_4 * bytes_per_pixel % 4) != 0) ?
				true : false;

	pix_per_line_added = (offset_from_4 * bytes_per_pixel % 4)
					/ bytes_per_pixel;
	if ((((fb_data->epdc_fb_var.rotate == FB_ROTATE_UR) ||
		fb_data->epdc_fb_var.rotate == FB_ROTATE_UD)) &&
		(ALIGN(src_upd_region->width, 8) <
			ALIGN(src_upd_region->width + pix_per_line_added, 8)))
		line_overflow = true;

	/*
	 * EPDC v2 returns the smallest collision rectangle in panel
	 * coordinates. Once transformed back to framebuffer coordinates that
	 * rectangle is not guaranteed to retain PxP's 8x8 alignment. Re-basing
	 * an unaligned retry in the existing white-padded copy buffer keeps PxP
	 * from reading beyond the collision rectangle without expanding the
	 * region eventually submitted to EPDC.
	 */
	collision_unaligned = upd_desc_list->collision_resubmit &&
		((src_upd_region->left | src_upd_region->top |
		  src_upd_region->width | src_upd_region->height) & 0x7);

	/* Grab pxp_mutex here so that we protect access
	 * to copybuf in addition to the PxP structures */
	mutex_lock(&fb_data->pxp_mutex);


	if (collision_unaligned ||
	    (((((width_unaligned || height_unaligned || input_unaligned) &&
		(upd_desc_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO))
		|| line_overflow) && (fb_data->rev < 20)) ||
		fb_data->restrict_width)) {
		dev_dbg(fb_data->dev, "Copying update before processing.\n");

		/* Update to reflect what the new source buffer will be */
		src_width = ALIGN(src_upd_region->width, 8);
		src_height = ALIGN(src_upd_region->height, 8);

		/*
		 * With an immutable snapshot, copy_before_process() no longer reads
		 * the Android framebuffer page. Keep the legacy diagnostic for paths
		 * where a snapshot is intentionally not used.
		 */
		if (!upd_desc_list->source_snapshot &&
		    epdc_fb_page_generation_changed(fb_data, upd_desc_list))
			epdc_record_fb_page_reuse(fb_data, true);
		copy_before_process(fb_data, upd_data_list);

		/*
		 * src_upd_region should now describe
		 * the new update buffer attributes.
		 */
		temp_buf_upd_region.left = 0;
		temp_buf_upd_region.top = 0;
		temp_buf_upd_region.width = src_upd_region->width;
		temp_buf_upd_region.height = src_upd_region->height;
		src_upd_region = &temp_buf_upd_region;

		use_temp_buf = true;
	}

	/*
	 * For pre-2.0 HW, input address must be 32-bit aligned
	 * Compute buffer offset to account for this PxP limitation
	 */
	offset_from_4 = src_upd_region->left & 0x3;
	input_unaligned = ((offset_from_4 * bytes_per_pixel % 4) != 0) ?
				true : false;
	if ((fb_data->rev < 20) && input_unaligned) {
		/* Leave a gap between PxP input addr and update region pixels */
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel & 0xFFFFFFFC;
		/* Update region left changes to reflect relative position to input ptr */
		pxp_upd_region.left = (offset_from_4 * bytes_per_pixel % 4)
					/ bytes_per_pixel;
	} else {
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel;
		pxp_upd_region.left = 0;
	}

	pxp_upd_region.top = 0;

	/*
	 * For version 2.0 and later of EPDC & PxP, if no rotation, we don't
	 * need to align width & height (rotation always requires 8-pixel
	 * width & height alignment, per PxP limitations)
	 */
	if ((fb_data->epdc_fb_var.rotate == 0) && (fb_data->rev >= 20)) {
		pxp_upd_region.width = src_upd_region->width;
		pxp_upd_region.height = src_upd_region->height;
	} else {
		/* Update region dimensions to meet 8x8 pixel requirement */
		pxp_upd_region.width = ALIGN(src_upd_region->width + pxp_upd_region.left, 8);
		pxp_upd_region.height = ALIGN(src_upd_region->height, 8);
	}

	switch (fb_data->epdc_fb_var.rotate) {
	case FB_ROTATE_UR:
	default:
		post_rotation_xcoord = pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.top;
		width_pxp_blocks = pxp_upd_region.width;
		break;
	case FB_ROTATE_CW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->height;
		post_rotation_ycoord = pxp_upd_region.left;
		break;
	case FB_ROTATE_UD:
		width_pxp_blocks = pxp_upd_region.width;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->width - pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.height - src_upd_region->height - pxp_upd_region.top;
		break;
	case FB_ROTATE_CCW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = pxp_upd_region.top;
		/*
		 * E70Q10 is 1404 pixels wide but PxP rotation works in 8-pixel
		 * blocks. The input is therefore processed as 1408 pixels wide;
		 * x=1404..1407 is white padding. ROT_270 maps those four padding
		 * columns to the first four output rows, so the EPDC must start at
		 * row 4 for a full-width update. Do not compact row 4 to row 0:
		 * output row 4 is logical source x=1403 (the visible right edge),
		 * while rows 0..3 are intentionally discarded padding.
		 */
		post_rotation_ycoord = pxp_upd_region.width - src_upd_region->width - pxp_upd_region.left;
		break;
	}

	if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_CW) ||
	    (fb_data->epdc_fb_var.rotate == FB_ROTATE_CCW))
		pxp_output_rows = pxp_upd_region.width;
	else
		pxp_output_rows = pxp_upd_region.height;
	pxp_output_bytes = width_pxp_blocks * pxp_output_rows;
	if (pxp_output_bytes > fb_data->max_pix_size)
		pxp_output_bytes = fb_data->max_pix_size;

	/* Update region start coord to force PxP to process full 8x8 regions */
	pxp_region_before_origin_align = pxp_upd_region;
	pxp_upd_region.top &= ~0x7;
	pxp_upd_region.left &= ~0x7;

	if (fb_data->rev < 20) {
		pxp_output_shift = ALIGN(post_rotation_xcoord, 8)
			- post_rotation_xcoord;

		pxp_output_offs = post_rotation_ycoord * width_pxp_blocks
			+ pxp_output_shift;

		upd_desc_list->epdc_offs = ALIGN(pxp_output_offs, 8);
	} else {
		pxp_output_shift = 0;
		pxp_output_offs = post_rotation_ycoord * width_pxp_blocks
			+ post_rotation_xcoord;

		upd_desc_list->epdc_offs = pxp_output_offs;
	}

	upd_desc_list->epdc_stride = width_pxp_blocks;

	/* Source address either comes from alternate buffer
	   provided in update data, or from the framebuffer. */
	if (use_temp_buf)
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->phys_addr_copybuf;
	else if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)
		sg_dma_address(&fb_data->sg[0]) =
			upd_desc_list->upd_data.alt_buffer_data.phys_addr
				+ pxp_input_offs;
	else if (upd_desc_list->source_snapshot) {
		sg_dma_address(&fb_data->sg[0]) =
			upd_desc_list->source_snapshot->phys_addr + pxp_input_offs;
		sg_set_page(&fb_data->sg[0],
			virt_to_page(upd_desc_list->source_snapshot->virt_addr),
			fb_data->source_snapshot_size,
			offset_in_page(upd_desc_list->source_snapshot->virt_addr));
	} else {
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->info.fix.smem_start + upd_desc_list->fb_offset
			+ pxp_input_offs;
		sg_set_page(&fb_data->sg[0],
			virt_to_page(fb_data->info.screen_base),
			fb_data->info.fix.smem_len,
			offset_in_page(fb_data->info.screen_base));
	}

	/* Update sg[1] to point to output of PxP proc task */
	sg_dma_address(&fb_data->sg[1]) = upd_data_list->phys_addr
						+ pxp_output_shift;
	sg_set_page(&fb_data->sg[1], virt_to_page(upd_data_list->virt_addr),
		    fb_data->max_pix_size,
		    offset_in_page(upd_data_list->virt_addr));

	/*
	 * Set PxP LUT transform type based on update flags.
	 */
	fb_data->pxp_conf.proc_data.lut_transform = 0;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_ENABLE_INVERSION)
		fb_data->pxp_conf.proc_data.lut_transform |= PXP_LUT_INVERT;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_FORCE_MONOCHROME)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_BLACK_WHITE;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_CMAP)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_USE_CMAP;

	/*
	 * Toggle inversion processing if 8-bit
	 * inverted is the current pixel format.
	 */
	if (fb_data->epdc_fb_var.grayscale == GRAYSCALE_8BIT_INVERTED)
		fb_data->pxp_conf.proc_data.lut_transform ^= PXP_LUT_INVERT;

	/* Netronix P5N waveforms consume PxP-quantized 5-bit pixel values. */
	if (fb_data->buf_pix_fmt == EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N)
		fb_data->pxp_conf.proc_data.lut_transform |= PXP_LUT_AA;


	/* The temp/alternate source is no longer the framebuffer page. */
	ret = pxp_process_update(fb_data, src_width, src_height,
		&pxp_upd_region,
		(use_temp_buf || upd_desc_list->source_snapshot ||
		 (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)) ?
			NULL : upd_desc_list);
	if (ret) {
		dev_err(fb_data->dev, "Unable to submit PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	/* If needed, enable EPDC HW while ePxP is processing */
	if ((fb_data->power_state == POWER_STATE_OFF)
		|| fb_data->powering_down) {
		epdc_powerup(fb_data);
	}

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_complete_update(fb_data, &hist_stat);
	if (ret) {
		dev_err(fb_data->dev, "Unable to complete PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}




	mutex_unlock(&fb_data->pxp_mutex);

	/* Update waveform mode from PxP histogram results */
	if ((fb_data->rev <= 20) &&
		(upd_desc_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO)) {
		if (hist_stat & 0x1)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_du;
		else if (hist_stat & 0x2)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc4;
		else if (hist_stat & 0x4)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc8;
		else if (hist_stat & 0x8)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc16;
		else
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc32;

		dev_dbg(fb_data->dev, "hist_stat = 0x%x, new waveform = 0x%x\n",
			hist_stat, upd_desc_list->upd_data.waveform_mode);
	}

	return 0;
}

static int epdc_submit_merge(struct update_desc_list *upd_desc_list,
				struct update_desc_list *update_to_merge,
				struct mxc_epdc_fb_data *fb_data)
{
	struct mxcfb_update_data *a, *b;
	struct mxcfb_rect *arect, *brect;
	struct mxcfb_rect combine;
	bool use_flags = false;

	a = &upd_desc_list->upd_data;
	b = &update_to_merge->upd_data;
	arect = &upd_desc_list->upd_data.update_region;
	brect = &update_to_merge->upd_data.update_region;

	/*
	 * A full-panel GC16 is a physical cleanup, not just another update
	 * candidate. Merging it with another update can turn its waveform into AUTO
	 * below and can also change the rectangle that the marker represents. Keep it
	 * as a queue barrier in either merge direction.
	 */
	if (epdc_is_full_panel_gc16_cleanup(fb_data, a) ||
		epdc_is_full_panel_gc16_cleanup(fb_data, b)) {
		return MERGE_BLOCK;
	}

	/* Do not merge a dry-run collision test update */
	if ((a->flags & EPDC_FLAG_TEST_COLLISION) ||
		(b->flags & EPDC_FLAG_TEST_COLLISION)) {
		return MERGE_BLOCK;
	}

	/*
	 * Updates with different flags must be executed sequentially.
	 * Halt the merge process to ensure this.
	 */
	if (a->flags != b->flags) {
		/*
		 * Special exception: if update regions are identical,
		 * we may be able to merge them.
		 */
		if ((arect->left != brect->left) ||
			(arect->top != brect->top) ||
			(arect->width != brect->width) ||
			(arect->height != brect->height)) {
			return MERGE_BLOCK;
		}

		use_flags = true;
	}

	if (a->update_mode != b->update_mode)
		a->update_mode = UPDATE_MODE_FULL;

	if (a->waveform_mode != b->waveform_mode)
		a->waveform_mode = WAVEFORM_MODE_AUTO;

	if (arect->left > (brect->left + brect->width) ||
		brect->left > (arect->left + arect->width) ||
		arect->top > (brect->top + brect->height) ||
		brect->top > (arect->top + arect->height)) {
		return MERGE_FAIL;
	}

	combine.left = arect->left < brect->left ? arect->left : brect->left;
	combine.top = arect->top < brect->top ? arect->top : brect->top;
	combine.width = (arect->left + arect->width) >
			(brect->left + brect->width) ?
			(arect->left + arect->width - combine.left) :
			(brect->left + brect->width - combine.left);
	combine.height = (arect->top + arect->height) >
			(brect->top + brect->height) ?
			(arect->top + arect->height - combine.top) :
			(brect->top + brect->height - combine.top);

	/* Don't merge if combined width exceeds max width */
	if (fb_data->restrict_width) {
		u32 max_width = EPDC_V2_MAX_UPDATE_WIDTH;
		u32 combined_width = combine.width;
		if (fb_data->epdc_fb_var.rotate != FB_ROTATE_UR)
			max_width -= EPDC_V2_ROTATION_ALIGNMENT;
		if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_CW) ||
			(fb_data->epdc_fb_var.rotate == FB_ROTATE_CCW))
			combined_width = combine.height;
		if (combined_width > max_width) {
			return MERGE_FAIL;
		}
	}

	*arect = combine;

	/* Use flags of the later update */
	if (use_flags)
		a->flags = b->flags;

	/* Merge markers */
	list_splice_tail(&update_to_merge->upd_marker_list,
		&upd_desc_list->upd_marker_list);

	/* Merged pixels must come from the newest framebuffer snapshot. */
	if (update_to_merge->update_order >= upd_desc_list->update_order) {
		upd_desc_list->fb_offset = update_to_merge->fb_offset;
		upd_desc_list->fb_source_generation =
			update_to_merge->fb_source_generation;
		upd_desc_list->fb_page_index = update_to_merge->fb_page_index;
		upd_desc_list->fb_page_generation =
			update_to_merge->fb_page_generation;
		upd_desc_list->fb_page_generation_valid =
			update_to_merge->fb_page_generation_valid;
		e70q10_source_snapshot_adopt_newer_locked(fb_data,
			upd_desc_list, update_to_merge);
	}
	upd_desc_list->merged = true;
	upd_desc_list->collision_resubmit |=
		update_to_merge->collision_resubmit;
	upd_desc_list->deferred_retry |= update_to_merge->deferred_retry;
	upd_desc_list->collided_newer_order = max(
		upd_desc_list->collided_newer_order,
		update_to_merge->collided_newer_order);
	upd_desc_list->collided_newer_seen |=
		update_to_merge->collided_newer_seen;
	upd_desc_list->collision_retry_count = max(
		upd_desc_list->collision_retry_count,
		update_to_merge->collision_retry_count);
	upd_desc_list->update_order =
		(upd_desc_list->update_order > update_to_merge->update_order) ?
		upd_desc_list->update_order : update_to_merge->update_order;
	return MERGE_OK;
}

/* Keep a selected descriptor reachable while recovery waits for this worker. */
static void epdc_recovery_park_update_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_data_list *update)
{
	if (update && list_empty(&update->list))
		list_add_tail(&update->list, &fb_data->upd_buf_queue);
}

static void epdc_recovery_abort_selected_locked(
		struct mxc_epdc_fb_data *fb_data,
		struct update_data_list *update)
{
	if (atomic_read(&fb_data->epdc_recovery_state) == 2) {
		bool had_alt_buffer = false;

		epdc_recovery_fail_desc_locked(fb_data, update->update_desc,
						 &had_alt_buffer);
		update->update_desc = NULL;
		if (list_empty(&update->list))
			list_add_tail(&update->list, &fb_data->upd_buf_free_list);
		return;
	}
	epdc_recovery_park_update_locked(fb_data, update);
}

static void epdc_submit_work_func(struct work_struct *work)
{
	int temp_index;
	struct update_data_list *next_update, *temp_update;
	struct update_desc_list *next_desc, *temp_desc;
	struct update_marker_data *next_marker, *temp_marker;
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_submit_work);
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect adj_update_region, *upd_region;
	bool end_merge = false;
	bool is_transform;
	u32 update_addr;
	int *err_dist;
	int ret;

	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);
	if (atomic_read(&fb_data->epdc_recovery_state) &&
	    !fb_data->recovery_refresh_active) {
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}


	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */
	list_for_each_entry_safe(next_update, temp_update,
				&fb_data->upd_buf_collision_list, list) {

		if (next_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");
		fb_data->collision_retry_zero_mask++;
		next_update->collision_mask = 0;
		next_update->update_desc->collision_retry_count++;
		fb_data->collision_retry_max_depth = max_t(u64,
			fb_data->collision_retry_max_depth,
			next_update->update_desc->collision_retry_count);
		next_update->update_desc->collision_resubmit = true;
		/*
		 * Histogram-selected AUTO is useful for ordinary collision retries,
		 * but a full-panel GC16 cleanup must retain the waveform requested by
		 * userspace all the way to the hardware submission.
		 */
		if (!epdc_is_cleanup_desc(fb_data, next_update->update_desc))
			next_update->update_desc->upd_data.waveform_mode =
				WAVEFORM_MODE_AUTO;
		/*
		 * We have a collision cleared, so select it for resubmission.
		 * If an update is already selected, attempt to merge.
		 */
		if (!upd_data_list) {
			upd_data_list = next_update;
			list_del_init(&next_update->list);
			if (fb_data->upd_scheme == UPDATE_SCHEME_QUEUE)
				/* If not merging, we have our update */
				break;
		} else {
				switch (epdc_submit_merge(upd_data_list->update_desc,
							next_update->update_desc,
							fb_data)) {
			case MERGE_OK:
				dev_dbg(fb_data->dev,
					"Update merged [collision]\n");
				list_del_init(&next_update->update_desc->list);
				epdc_free_update_desc_locked(fb_data, next_update->update_desc);
				next_update->update_desc = NULL;
				list_del_init(&next_update->list);
				/* Add to free buffer list */
				list_add_tail(&next_update->list,
					 &fb_data->upd_buf_free_list);
				break;
			case MERGE_FAIL:
				dev_dbg(fb_data->dev,
					"Update not merged [collision]\n");
				break;
			case MERGE_BLOCK:
				dev_dbg(fb_data->dev,
					"Merge blocked [collision]\n");
				end_merge = true;
				break;
			}

			if (end_merge) {
				end_merge = false;
				break;
			}
		}
	}

	/*
	 * Skip pending update list only if we found a collision
	 * update and we are not merging
	 */
	if (!((fb_data->upd_scheme == UPDATE_SCHEME_QUEUE) &&
		upd_data_list)) {
		/*
		 * If we didn't find a collision update ready to go, we
		 * need to get a free buffer and match it to a pending update.
		 */

		/*
		 * Can't proceed if there are no free buffers (and we don't
		 * already have a collision update selected)
		*/
		if (!upd_data_list &&
			list_empty(&fb_data->upd_buf_free_list)) {
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}

		list_for_each_entry_safe(next_desc, temp_desc,
				&fb_data->upd_pending_list, list) {

			dev_dbg(fb_data->dev, "Found a pending update!\n");
			if (!e70q10_promote_deferred_snapshot_locked(fb_data,
								 next_desc))
				break;

			if (!upd_data_list) {
				if (list_empty(&fb_data->upd_buf_free_list))
					break;
				upd_data_list =
					list_entry(fb_data->upd_buf_free_list.next,
						struct update_data_list, list);
				list_del_init(&upd_data_list->list);
				upd_data_list->update_desc = next_desc;
				list_del_init(&next_desc->list);
				upd_data_list->collision_mask = 0;
				if (fb_data->upd_scheme == UPDATE_SCHEME_QUEUE)
					/* If not merging, we have an update */
					break;
			} else {
				switch (epdc_submit_merge(upd_data_list->update_desc,
						next_desc, fb_data)) {
				case MERGE_OK:
					dev_dbg(fb_data->dev,
						"Update merged [queue]\n");
					list_del_init(&next_desc->list);
					epdc_free_update_desc_locked(fb_data, next_desc);
					break;
				case MERGE_FAIL:
					dev_dbg(fb_data->dev,
						"Update not merged [queue]\n");
					break;
				case MERGE_BLOCK:
					dev_dbg(fb_data->dev,
						"Merge blocked [collision]\n");
					end_merge = true;
					break;
				}

				if (end_merge)
					break;
			}
		}
	}

	/* Is update list empty? */
	if (!upd_data_list) {
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}
	upd_data_list->pxp_used = false;

	/*
	 * If no processing required, skip update processing
	 * No processing means:
	 *   - FB unrotated
	 *   - FB pixel format = 8-bit grayscale
	 *   - No look-up transformations (inversion, posterization, etc.)
	 *
	 * Note: A bug with EPDC stride prevents us from skipping
	 * PxP in versions 2.0 and earlier of EPDC.
	 */
	is_transform = upd_data_list->update_desc->upd_data.flags &
		(EPDC_FLAG_ENABLE_INVERSION | EPDC_FLAG_USE_DITHERING_Y1 |
		EPDC_FLAG_USE_DITHERING_Y4 | EPDC_FLAG_FORCE_MONOCHROME |
		EPDC_FLAG_USE_CMAP) ? true : false;

	if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_UR) &&
		(fb_data->epdc_fb_var.grayscale == GRAYSCALE_8BIT) &&
		!is_transform && (fb_data->rev > 20) &&
		!fb_data->restrict_width) {

		/* If needed, enable EPDC HW while ePxP is processing */
		if ((fb_data->power_state == POWER_STATE_OFF)
			|| fb_data->powering_down)
			epdc_powerup(fb_data);

		/*
		 * Set update buffer pointer to the start of
		 * the update region in the frame buffer.
		 */
		upd_region = &upd_data_list->update_desc->upd_data.update_region;
		if (upd_data_list->update_desc->source_snapshot)
			update_addr = upd_data_list->update_desc->source_snapshot->phys_addr +
				((upd_region->top * fb_data->info.var.xres_virtual) +
				 upd_region->left) *
				fb_data->info.var.bits_per_pixel/8;
		else
			update_addr = fb_data->info.fix.smem_start +
				((upd_region->top * fb_data->info.var.xres_virtual) +
				 upd_region->left) *
				fb_data->info.var.bits_per_pixel/8;
		upd_data_list->update_desc->epdc_stride =
					fb_data->info.var.xres_virtual *
					fb_data->info.var.bits_per_pixel/8;
	} else {
		/* Select from PxP output buffers */
		upd_data_list->pxp_used = true;
		upd_data_list->phys_addr =
			fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
		upd_data_list->virt_addr =
			fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
		fb_data->upd_buffer_num++;
		if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
			fb_data->upd_buffer_num = 0;

		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);

		/* Perform PXP processing - EPDC power will also be enabled */
		ret = epdc_process_update(upd_data_list, fb_data);
		mutex_lock(&fb_data->queue_mutex);
		if (atomic_read(&fb_data->epdc_recovery_state) &&
		    !fb_data->recovery_refresh_active) {
			epdc_recovery_abort_selected_locked(fb_data,
							   upd_data_list);
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}
		if (ret) {
			dev_dbg(fb_data->dev, "PXP processing error.\n");
			list_for_each_entry_safe(next_marker, temp_marker,
					 &upd_data_list->update_desc->upd_marker_list,
					 upd_list)
				epdc_recovery_fail_marker_locked(fb_data, next_marker);
			if (epdc_is_cleanup_desc(fb_data,
					upd_data_list->update_desc)) {
				fb_data->cleanup_fence_active = false;
				fb_data->cleanup_fence_lut = INVALID_LUT;
			}
			if (upd_data_list->update_desc->deferred_retry)
				fb_data->deferred_latest_retry_dropped_error++;
			list_del_init(&upd_data_list->update_desc->list);
			epdc_free_update_desc_locked(fb_data, upd_data_list->update_desc);
			upd_data_list->update_desc = NULL;
			/* Add to free buffer list */
			list_add_tail(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
			/* Release buffer queues */
			mutex_unlock(&fb_data->queue_mutex);
			/* A freed snapshot can unblock the one-deep latest-state latch. */
			queue_work(fb_data->epdc_submit_workqueue,
				   &fb_data->epdc_submit_work);
			return;
		}

		update_addr = upd_data_list->phys_addr
				+ upd_data_list->update_desc->epdc_offs;
	}

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data->epdc_fb_var.xres,
		fb_data->epdc_fb_var.yres, fb_data->epdc_fb_var.rotate,
		&upd_data_list->update_desc->upd_data.update_region,
		&adj_update_region);

	/*
	 * Is the working buffer idle?
	 * If the working buffer is busy, we must wait for the resource
	 * to become free. The IST will signal this event.
	 */
	if (fb_data->cur_update != NULL) {
		dev_dbg(fb_data->dev, "working buf busy!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_wb = true;

		/* Leave spinlock while waiting for WB to complete */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->update_res_free);
		mutex_lock(&fb_data->queue_mutex);
		if (atomic_read(&fb_data->epdc_recovery_state) &&
		    !fb_data->recovery_refresh_active) {
			epdc_recovery_abort_selected_locked(fb_data,
							   upd_data_list);
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}
	}

	/*
	 * Dithering Processing (Version 1.0 - for i.MX508 and i.MX6SL)
	 */
	if (upd_data_list->update_desc->upd_data.flags &
	    EPDC_FLAG_USE_DITHERING_Y1) {

		err_dist = kzalloc((fb_data->info.var.xres_virtual + 3) * 3
				* sizeof(int), GFP_KERNEL);

		/* Dithering Y8 -> Y1 */
		do_dithering_processing_Y1_v1_0(
				(uint8_t *)(upd_data_list->virt_addr +
				upd_data_list->update_desc->epdc_offs),
				upd_data_list->phys_addr +
				upd_data_list->update_desc->epdc_offs,
				&adj_update_region,
				(fb_data->rev < 20) ?
				ALIGN(adj_update_region.width, 8) :
				adj_update_region.width,
				err_dist);

		kfree(err_dist);
	} else if (upd_data_list->update_desc->upd_data.flags &
		EPDC_FLAG_USE_DITHERING_Y4) {

		err_dist = kzalloc((fb_data->info.var.xres_virtual + 3) * 3
				* sizeof(int), GFP_KERNEL);

		/* Dithering Y8 -> Y1 */
		do_dithering_processing_Y4_v1_0(
				(uint8_t *)(upd_data_list->virt_addr +
				upd_data_list->update_desc->epdc_offs),
				upd_data_list->phys_addr +
				upd_data_list->update_desc->epdc_offs,
				&adj_update_region,
				(fb_data->rev < 20) ?
				ALIGN(adj_update_region.width, 8) :
				adj_update_region.width,
				err_dist);

		kfree(err_dist);
	}

	/*
	 * If there are no LUTs available,
	 * then we must wait for the resource to become free.
	 * The IST will signal this event.
	 */
	if (!epdc_any_luts_available()) {
		dev_dbg(fb_data->dev, "no luts available!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_lut = true;

		/* Leave spinlock while waiting for LUT to free up */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->update_res_free);
		mutex_lock(&fb_data->queue_mutex);
		if (atomic_read(&fb_data->epdc_recovery_state) &&
		    !fb_data->recovery_refresh_active) {
			epdc_recovery_abort_selected_locked(fb_data,
							   upd_data_list);
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}
	}

	ret = epdc_choose_next_lut(fb_data->rev, &upd_data_list->lut_num);
	/*
	 * If LUT15 is in use (for pre-EPDC v2.0 hardware):
	 *   - Wait for LUT15 to complete is if TCE underrun prevent is enabled
	 *   - If we go ahead with update, sync update submission with EOF
	 */
	if (ret && fb_data->tce_prevent && (fb_data->rev < 20)) {
		dev_dbg(fb_data->dev, "Waiting for LUT15\n");

		/* Initialize event signalling that lut15 is free */
		init_completion(&fb_data->lut15_free);

		fb_data->waiting_for_lut15 = true;

		/* Leave spinlock while waiting for LUT to free up */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->lut15_free);
		mutex_lock(&fb_data->queue_mutex);
		if (atomic_read(&fb_data->epdc_recovery_state) &&
		    !fb_data->recovery_refresh_active) {
			epdc_recovery_abort_selected_locked(fb_data,
							   upd_data_list);
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}

		epdc_choose_next_lut(fb_data->rev, &upd_data_list->lut_num);
	} else if (ret && (fb_data->rev < 20)) {
		/* Synchronize update submission time to reduce
		   chances of TCE underrun */
		init_completion(&fb_data->eof_event);

		epdc_eof_intr(true);

		/* Leave spinlock while waiting for EOF event */
		mutex_unlock(&fb_data->queue_mutex);
		ret = wait_for_completion_timeout(&fb_data->eof_event,
			msecs_to_jiffies(1000));
		if (!ret) {
			dev_err(fb_data->dev, "Missed EOF event!\n");
			epdc_eof_intr(false);
		}
		udelay(fb_data->eof_sync_period);
		mutex_lock(&fb_data->queue_mutex);
		if (atomic_read(&fb_data->epdc_recovery_state) &&
		    !fb_data->recovery_refresh_active) {
			epdc_recovery_abort_selected_locked(fb_data,
							   upd_data_list);
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}

	}

	/* LUTs are available, so we get one here */
	if (atomic_read(&fb_data->epdc_recovery_state) &&
	    !fb_data->recovery_refresh_active) {
		epdc_recovery_abort_selected_locked(fb_data, upd_data_list);
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}
	fb_data->cur_update = upd_data_list;
	if (epdc_is_cleanup_desc(fb_data, upd_data_list->update_desc))
		fb_data->cleanup_fence_lut = upd_data_list->lut_num;

	/* Reset mask for LUTS that have completed during WB processing */
	fb_data->luts_complete_wb = 0;

	/* If we are just testing for collision, we don't assign a LUT,
	 * so we don't need to update LUT-related resources. */
	if (!(upd_data_list->update_desc->upd_data.flags
		& EPDC_FLAG_TEST_COLLISION)) {
		/* Associate LUT with update marker */
		list_for_each_entry_safe(next_marker, temp_marker,
			&upd_data_list->update_desc->upd_marker_list, upd_list)
			next_marker->lut_num = fb_data->cur_update->lut_num;
		if (upd_data_list->update_desc->sleep_sequence) {
			fb_data->sleep_lut_sequence[upd_data_list->lut_num] =
				upd_data_list->update_desc->sleep_sequence;
		}

		/* Mark LUT with order */
		fb_data->lut_update_order[upd_data_list->lut_num] =
			upd_data_list->update_desc->update_order;

		epdc_lut_complete_intr(fb_data->rev, upd_data_list->lut_num,
					true);
	}

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);

	/* Program EPDC update to process buffer */
	if (upd_data_list->update_desc->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			upd_data_list->update_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	epdc_set_update_addr(update_addr);
	epdc_set_update_coord(adj_update_region.left, adj_update_region.top);
	epdc_set_update_dimensions(adj_update_region.width,
				   adj_update_region.height);
	if (fb_data->rev > 20)
		epdc_set_update_stride(upd_data_list->update_desc->epdc_stride);
	if (fb_data->wv_modes_update &&
		(upd_data_list->update_desc->upd_data.waveform_mode
			== WAVEFORM_MODE_AUTO)) {
		epdc_set_update_waveform(&fb_data->wv_modes);
		fb_data->wv_modes_update = false;
	}

		if (upd_data_list->update_desc->collision_resubmit)
			fb_data->collision_retry_submitted++;
		if (upd_data_list->update_desc->deferred_retry)
			fb_data->deferred_latest_retry_submitted++;
		fb_data->latest_submitted_generation =
			upd_data_list->update_desc->fb_source_generation;
		epdc_submit_update(fb_data, upd_data_list->lut_num,
			   upd_data_list->update_desc->upd_data.waveform_mode,
			   upd_data_list->update_desc->upd_data.update_mode,
			   (upd_data_list->update_desc->upd_data.flags
				& EPDC_FLAG_TEST_COLLISION) ? true : false,
			   false, 0);
	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);
}

static int __mxc_epdc_fb_send_single_update(
				   struct mxcfb_update_data *upd_data,
				   struct fb_info *info, bool serialize,
				   void *serialize_wait_state,
				   u32 sleep_sequence)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect *screen_upd_region; /* Region on screen to update */
	int temp_index;
	int ret;
	struct update_desc_list *upd_desc;
	struct update_marker_data *marker_data, *next_marker, *temp_marker;
	struct epdc_pending_coalesce_plan coalesce_plan;
	bool cleanup_refresh;
	bool recovery_update = serialize;
	bool hw_wb_busy;
	bool hw_luts_active;
	bool coalesce_plan_valid;
	(void)serialize_wait_state;

	/* Has EPDC HW been initialized? */
	if (!fb_data->hw_ready) {
		/* Throw message if we are not mid-initialization */
		if (!fb_data->hw_initializing)
			dev_err(fb_data->dev, "Display HW not properly"
				"initialized. Aborting update.\n");
		return -EPERM;
	}

	/* Check validity of update params */
	if ((upd_data->update_mode != UPDATE_MODE_PARTIAL) &&
		(upd_data->update_mode != UPDATE_MODE_FULL)) {
		dev_err(fb_data->dev,
			"Update mode 0x%x is invalid.  Aborting update.\n",
			upd_data->update_mode);
		return -EINVAL;
	}
	if ((upd_data->waveform_mode > 255) &&
		(upd_data->waveform_mode != WAVEFORM_MODE_AUTO)) {
		dev_err(fb_data->dev,
			"Update waveform mode 0x%x is invalid."
			"  Aborting update.\n",
			upd_data->waveform_mode);
		return -EINVAL;
	}
	cleanup_refresh = epdc_is_full_panel_gc16_cleanup(fb_data, upd_data);

	mutex_lock(&fb_data->queue_mutex);
	if ((upd_data->update_region.left + upd_data->update_region.width > fb_data->epdc_fb_var.xres) ||
		(upd_data->update_region.top + upd_data->update_region.height > fb_data->epdc_fb_var.yres)) {
		mutex_unlock(&fb_data->queue_mutex);
		dev_err(fb_data->dev,
			"Update region is outside bounds of framebuffer."
			"Aborting update.\n");
		return -EINVAL;
	}
	mutex_unlock(&fb_data->queue_mutex);

	if (upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) {
		if ((upd_data->update_region.width !=
			upd_data->alt_buffer_data.alt_update_region.width) ||
			(upd_data->update_region.height !=
			upd_data->alt_buffer_data.alt_update_region.height)) {
			dev_err(fb_data->dev,
				"Alternate update region dimensions must "
				"match screen update region dimensions.\n");
			return -EINVAL;
		}
		/* Validate physical address parameter */
		if ((upd_data->alt_buffer_data.phys_addr <
			fb_data->info.fix.smem_start) ||
			(upd_data->alt_buffer_data.phys_addr >
			fb_data->info.fix.smem_start + fb_data->map_size)) {
			dev_err(fb_data->dev,
				"Invalid physical address for alternate "
				"buffer.  Aborting update...\n");
			return -EINVAL;
		}
	}

	mutex_lock(&fb_data->queue_mutex);
	if (fb_data->cleanup_fence_active) {
		mutex_unlock(&fb_data->queue_mutex);
		return -EBUSY;
	}

	/*
	 * If we are waiting to go into suspend, or the FB is blanked,
	 * we do not accept new updates
	 */
	if ((!recovery_update &&
	     (fb_data->waiting_for_idle || fb_data->suspend_blocked ||
	      fb_data->sleep_barrier_waiting ||
	      atomic_read(&fb_data->epdc_recovery_state))) ||
	    (fb_data->blank != FB_BLANK_UNBLANK)) {
		dev_dbg(fb_data->dev, "EPDC not active."
			"Update request abort.\n");
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}

	/*
	 * QUEUE_AND_MERGE normally allows several LUTs to overlap.  A cleanup
	 * refresh instead starts only after every older pending/active update has
	 * completed. MXCFB_SEND_UPDATE holds hard_lock around this function, so a
	 * second userspace submission cannot slip between this flush and enqueue.
	 */
	if (cleanup_refresh) {
		mutex_unlock(&fb_data->queue_mutex);
		ret = mxc_epdc_fb_flush_updates(fb_data);
		mutex_lock(&fb_data->queue_mutex);
		hw_wb_busy = fb_data->power_state == POWER_STATE_ON &&
			epdc_is_working_buffer_busy();
		hw_luts_active = fb_data->power_state == POWER_STATE_ON &&
			epdc_any_luts_active(fb_data->rev);
		/*
		 * Do not trust updates_active alone here.  The EPDC may still have a
		 * working-buffer transaction or a panel LUT active while the software
		 * lists momentarily look empty.
		 */
		if (ret || !list_empty(&fb_data->upd_pending_list) ||
		    !list_empty(&fb_data->upd_buf_collision_list) ||
		    !is_free_list_full(fb_data) || fb_data->cur_update ||
		    fb_data->updates_active || hw_wb_busy || hw_luts_active) {
			mutex_unlock(&fb_data->queue_mutex);
			return ret ? ret : -EBUSY;
		}
	}


	if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
		int count = 0;
		struct update_data_list *plist;

		/*
		 * If next update is a FULL mode update, then we must
		 * ensure that all pending & active updates are complete
		 * before submitting the update.  Otherwise, the FULL
		 * mode update may cause an endless collision loop with
		 * other updates.  Block here until updates are flushed.
		 */
		if (upd_data->update_mode == UPDATE_MODE_FULL && !cleanup_refresh) {
			mutex_unlock(&fb_data->queue_mutex);
			mxc_epdc_fb_flush_updates(fb_data);
			mutex_lock(&fb_data->queue_mutex);
		}

		/* Count buffers in free buffer list */
		list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
			count++;

		/* Use count to determine if we have enough
		 * free buffers to handle this update request */
		if (count + fb_data->max_num_buffers
			<= fb_data->max_num_updates) {
			dev_err(fb_data->dev,
				"No free intermediate buffers available.\n");
			mutex_unlock(&fb_data->queue_mutex);
			return -ENOMEM;
		}

		/* Grab first available buffer and delete from the free list */
		upd_data_list =
		    list_entry(fb_data->upd_buf_free_list.next,
			       struct update_data_list, list);

		list_del_init(&upd_data_list->list);
	}

	if (cleanup_refresh) {
		fb_data->cleanup_fence_active = true;
		fb_data->cleanup_fence_lut = INVALID_LUT;
		fb_data->cleanup_fence_marker = upd_data->update_marker;
	}

	/*
	 * Create new update data structure, fill it with new update
	 * data and add it to the list of pending updates
	 */
	upd_desc = kzalloc(sizeof(struct update_desc_list), GFP_KERNEL);
	if (!upd_desc) {
		dev_err(fb_data->dev,
			"Insufficient system memory for update! Aborting.\n");
		if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
			list_add(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
		}
		if (cleanup_refresh) {
			fb_data->cleanup_fence_active = false;
			fb_data->cleanup_fence_lut = INVALID_LUT;
		}
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}
	/* Initialize per-update marker list */
	INIT_LIST_HEAD(&upd_desc->upd_marker_list);
	upd_desc->upd_data = *upd_data;
	upd_desc->sleep_sequence = sleep_sequence;
	/*
	 * FBIOPAN_DISPLAY changes fb_offset for every SurfaceFlinger target.
	 * Queue processing is asynchronous, so using the later global value in
	 * epdc_process_update can read a different triple-buffer page than the
	 * one associated with this request. Capture it under queue_mutex.
	 */
	upd_desc->fb_offset = fb_data->fb_offset;
	upd_desc->fb_source_generation = fb_data->fb_source_generation;
	upd_desc->fb_page_index = -1;
	upd_desc->fb_page_generation_valid = false;
	if (fb_data->page_lifetime_diag &&
	    !(upd_desc->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)) {
		unsigned long page_flags;
		int page = epdc_fb_page_index(fb_data, upd_desc->fb_offset);

		if (page >= 0) {
			spin_lock_irqsave(&fb_data->fb_page_lock, page_flags);
			upd_desc->fb_page_index = page;
			upd_desc->fb_page_generation =
				fb_data->fb_page_generations[page];
			upd_desc->fb_page_generation_valid = true;
			spin_unlock_irqrestore(&fb_data->fb_page_lock, page_flags);
		} else {
			spin_lock_irqsave(&fb_data->fb_page_lock, page_flags);
			fb_data->untracked_fb_submit++;
			spin_unlock_irqrestore(&fb_data->fb_page_lock, page_flags);
		}
	}

	upd_desc->update_order = fb_data->order_cnt++;
	coalesce_plan_valid = epdc_plan_pending_coalesce_locked(fb_data,
		upd_desc, &coalesce_plan);
	ret = e70q10_source_snapshot_acquire_locked(fb_data, upd_desc);
	if (ret == -EBUSY && coalesce_plan_valid &&
	    epdc_reuse_pending_snapshot_locked(fb_data, upd_desc,
					       &coalesce_plan))
		ret = 0;
	if (ret == -EBUSY) {
		fb_data->source_snapshot_busy++;
		if (epdc_is_ordinary_ui_update(fb_data, upd_desc, false)) {
			fb_data->snapshot_busy_final_candidate++;
			if (coalesce_plan_valid &&
			    e70q10_defer_latest_update_locked(fb_data, upd_desc,
								     &coalesce_plan)) {
				mutex_unlock(&fb_data->queue_mutex);
				queue_work(fb_data->epdc_submit_workqueue,
					   &fb_data->epdc_submit_work);
				return 0;
			}
			fb_data->deferred_latest_retry_dropped_error++;
		}
	}
	if (ret) {
		epdc_free_update_desc_locked(fb_data, upd_desc);
		if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT)
			list_add_tail(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
		if (cleanup_refresh) {
			fb_data->cleanup_fence_active = false;
			fb_data->cleanup_fence_lut = INVALID_LUT;
		}
		mutex_unlock(&fb_data->queue_mutex);
		return ret;
	}

	if (coalesce_plan_valid)
		epdc_apply_pending_coalesce_locked(fb_data, upd_desc,
						   &coalesce_plan);
	list_add_tail(&upd_desc->list, &fb_data->upd_pending_list);
	/* If marker specified, associate it with a completion */
	if (upd_data->update_marker != 0) {
		/* Allocate new update marker and set it up */
		marker_data = kzalloc(sizeof(struct update_marker_data),
				GFP_KERNEL);
		if (!marker_data) {
			dev_err(fb_data->dev, "No memory for marker!\n");
			list_del_init(&upd_desc->list);
			epdc_free_update_desc_locked(fb_data, upd_desc);
			if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT)
				list_add_tail(&upd_data_list->list,
					&fb_data->upd_buf_free_list);
			if (cleanup_refresh) {
				fb_data->cleanup_fence_active = false;
				fb_data->cleanup_fence_lut = INVALID_LUT;
			}
			mutex_unlock(&fb_data->queue_mutex);
			queue_work(fb_data->epdc_submit_workqueue,
				   &fb_data->epdc_submit_work);
			return -ENOMEM;
		}
		list_add_tail(&marker_data->upd_list,
			&upd_desc->upd_marker_list);
		marker_data->update_marker = upd_data->update_marker;
		marker_data->sleep_sequence = sleep_sequence;
		marker_data->update_generation = upd_desc->update_order;
		marker_data->retained_sleep = !!sleep_sequence;
		marker_data->waiting = marker_data->retained_sleep;
		INIT_LIST_HEAD(&marker_data->sleep_done_list);
		if (upd_desc->upd_data.flags & EPDC_FLAG_TEST_COLLISION)
			marker_data->lut_num = DRY_RUN_NO_LUT;
		else
			marker_data->lut_num = INVALID_LUT;
		init_completion(&marker_data->update_completion);
		/* Add marker to master marker list */
		list_add_tail(&marker_data->full_list,
			&fb_data->full_marker_list);
	}
	fb_data->latest_accepted_generation = max(
		fb_data->latest_accepted_generation,
		upd_desc->fb_source_generation);

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		mutex_unlock(&fb_data->queue_mutex);

		/* Signal workqueue to handle new update */
		queue_work(fb_data->epdc_submit_workqueue,
			&fb_data->epdc_submit_work);

		return 0;
	}

	/* Snapshot update scheme processing */

	/* Set descriptor for current update, delete from pending list */
	upd_data_list->update_desc = upd_desc;
	list_del_init(&upd_desc->list);

	mutex_unlock(&fb_data->queue_mutex);

	/*
	 * Hold on to original screen update region, which we
	 * will ultimately use when telling EPDC where to update on panel
	 */
	screen_upd_region = &upd_desc->upd_data.update_region;

	/* Select from PxP output buffers */
	upd_data_list->phys_addr =
		fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
	upd_data_list->virt_addr =
		fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
	fb_data->upd_buffer_num++;
	if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
		fb_data->upd_buffer_num = 0;

	ret = epdc_process_update(upd_data_list, fb_data);
	if (ret) {
		mutex_lock(&fb_data->queue_mutex);
		list_for_each_entry_safe(next_marker, temp_marker,
				&upd_desc->upd_marker_list, upd_list) {
			epdc_recovery_fail_marker_locked(fb_data, next_marker);
		}
		epdc_free_update_desc_locked(fb_data, upd_desc);
		upd_data_list->update_desc = NULL;
		list_add_tail(&upd_data_list->list,
			&fb_data->upd_buf_free_list);
		if (cleanup_refresh) {
			fb_data->cleanup_fence_active = false;
			fb_data->cleanup_fence_lut = INVALID_LUT;
		}
		mutex_unlock(&fb_data->queue_mutex);
		return ret;
	}

	/* Pass selected waveform mode back to user */
	upd_data->waveform_mode = upd_desc->upd_data.waveform_mode;

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data->epdc_fb_var.xres,
		fb_data->epdc_fb_var.yres, fb_data->epdc_fb_var.rotate,
		&upd_desc->upd_data.update_region, NULL);

	/* Grab lock for queue manipulation and update submission */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * Is the working buffer idle?
	 * If either the working buffer is busy, or there are no LUTs available,
	 * then we return and let the ISR handle the update later
	 */
	if ((fb_data->cur_update != NULL) || !epdc_any_luts_available()) {
		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list, &fb_data->upd_buf_queue);

		/* Return and allow the update to be submitted by the ISR. */
		mutex_unlock(&fb_data->queue_mutex);
		return 0;
	}

	/* LUTs are available, so we get one here */
	ret = epdc_choose_next_lut(fb_data->rev, &upd_data_list->lut_num);
	if (ret && fb_data->tce_prevent && (fb_data->rev < 20)) {
		dev_dbg(fb_data->dev, "Must wait for LUT15\n");
		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list, &fb_data->upd_buf_queue);

		/* Return and allow the update to be submitted by the ISR. */
		mutex_unlock(&fb_data->queue_mutex);
		return 0;
	}

	if (!(upd_data_list->update_desc->upd_data.flags
		& EPDC_FLAG_TEST_COLLISION)) {

		/* Save current update */
		fb_data->cur_update = upd_data_list;
		if (epdc_is_cleanup_desc(fb_data, upd_data_list->update_desc))
			fb_data->cleanup_fence_lut = upd_data_list->lut_num;

		/* Reset mask for LUTS that have completed during WB processing */
		fb_data->luts_complete_wb = 0;

		/* Associate LUT with update marker */
		list_for_each_entry_safe(next_marker, temp_marker,
			&upd_data_list->update_desc->upd_marker_list, upd_list)
			next_marker->lut_num = upd_data_list->lut_num;

		/* Mark LUT as containing new update */
		fb_data->lut_update_order[upd_data_list->lut_num] =
			upd_desc->update_order;

		epdc_lut_complete_intr(fb_data->rev, upd_data_list->lut_num,
					true);
	}

	/* Clear status and Enable LUT complete and WB complete IRQs */
	epdc_working_buf_intr(true);

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(upd_data_list->phys_addr + upd_desc->epdc_offs);
	epdc_set_update_coord(screen_upd_region->left, screen_upd_region->top);
	epdc_set_update_dimensions(screen_upd_region->width,
		screen_upd_region->height);
	if (fb_data->rev > 20)
		epdc_set_update_stride(upd_desc->epdc_stride);
	if (upd_desc->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			upd_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	if (fb_data->wv_modes_update &&
		(upd_desc->upd_data.waveform_mode == WAVEFORM_MODE_AUTO)) {
		epdc_set_update_waveform(&fb_data->wv_modes);
		fb_data->wv_modes_update = false;
	}
	epdc_submit_update(fb_data, upd_data_list->lut_num,
			   upd_desc->upd_data.waveform_mode,
			   upd_desc->upd_data.update_mode,
			   (upd_desc->upd_data.flags
				& EPDC_FLAG_TEST_COLLISION) ? true : false,
			   false, 0);

	mutex_unlock(&fb_data->queue_mutex);
	return 0;
}
static int mxc_epdc_fb_send_single_update(
		struct mxcfb_update_data *upd_data, struct fb_info *info,
		bool serialize, u32 sleep_sequence)
{
	return __mxc_epdc_fb_send_single_update(upd_data, info, serialize, NULL,
						 sleep_sequence);
}

static int __mxc_epdc_fb_send_update(
				   struct mxcfb_update_data *upd_data,
				   struct fb_info *info, bool serialize)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	u32 sleep_sequence = 0;
	bool tagged_sleep_update =
		!!(upd_data->flags & EPDC_FLAG_E70Q10_SLEEP_SCREEN);

	if (tagged_sleep_update) {
		sleep_sequence = EPDC_E70Q10_SLEEP_SEQUENCE(upd_data->flags);
		if (!sleep_sequence)
			return -EINVAL;
		upd_data->flags &= ~EPDC_E70Q10_SLEEP_TAG_MASK;
		mutex_lock(&fb_data->queue_mutex);
		if (!fb_data->sleep_barrier_powerdown_held)
			fb_data->sleep_barrier_saved_pwrdown_delay =
				fb_data->pwrdown_delay;
		fb_data->sleep_barrier_powerdown_held = true;
		fb_data->sleep_barrier_pending = true;
		fb_data->pwrdown_delay = FB_POWERDOWN_DISABLE;
		mutex_unlock(&fb_data->queue_mutex);
		/* A previous zero-delay shutdown may already be queued. */
		cancel_delayed_work_sync(&fb_data->epdc_done_work);
		mutex_lock(&fb_data->queue_mutex);
		epdc_sleep_marker_discard_old_locked(fb_data, sleep_sequence);
		fb_data->last_sleep_sequence = sleep_sequence;
		mutex_unlock(&fb_data->queue_mutex);
	}
	else {
		mutex_lock(&fb_data->queue_mutex);
		if (fb_data->sleep_barrier_powerdown_held &&
		    !fb_data->sleep_barrier_pending)
			epdc_sleep_barrier_restore_powerdown_locked(fb_data,
								   false);
		mutex_unlock(&fb_data->queue_mutex);
	}

	if (!fb_data->restrict_width) {
		/* No width restriction, send entire update region */
		return mxc_epdc_fb_send_single_update(upd_data, info, serialize,
						      sleep_sequence);
	} else {
		int ret;
		__u32 width, left;
		__u32 marker;
		__u32 *region_width, *region_left;
		u32 max_upd_width = EPDC_V2_MAX_UPDATE_WIDTH;

		/* Further restrict max width due to pxp rotation
		  * alignment requirement
		  */
		if (fb_data->epdc_fb_var.rotate != FB_ROTATE_UR)
			max_upd_width -= EPDC_V2_ROTATION_ALIGNMENT;

		/* Select split of width or height based on rotation */
		if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_UR) ||
			(fb_data->epdc_fb_var.rotate == FB_ROTATE_UD)) {
			region_width = &upd_data->update_region.width;
			region_left = &upd_data->update_region.left;
		} else {
			region_width = &upd_data->update_region.height;
			region_left = &upd_data->update_region.top;
		}

		if (*region_width <= max_upd_width)
			return mxc_epdc_fb_send_single_update(upd_data, info,
						      serialize, sleep_sequence);

		width = *region_width;
		left = *region_left;
		marker = upd_data->update_marker;
		upd_data->update_marker = 0;

		do {
			*region_width = max_upd_width;
			*region_left = left;
			ret = mxc_epdc_fb_send_single_update(upd_data, info,
							     serialize,
							     sleep_sequence);
			if (ret)
				return ret;
			width -= max_upd_width;
			left += max_upd_width;
		} while (width > max_upd_width);

		*region_width = width;
		*region_left = left;
		upd_data->update_marker = marker;
		return mxc_epdc_fb_send_single_update(upd_data, info, serialize,
					      sleep_sequence);
	}
}
int mxc_epdc_fb_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info)
{
	return __mxc_epdc_fb_send_update(upd_data, info, false);
}
EXPORT_SYMBOL(mxc_epdc_fb_send_update);

static int epdc_wait_update_marker_timeout(
		struct mxcfb_update_marker_data *marker_data,
		struct fb_info *info, unsigned long timeout, bool require_found,
		u32 required_sequence)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	bool marker_found = false;
	bool completed_marker_found = false;
	int ret = 0;

	/* 0 is an invalid update_marker value */
	if (marker_data->update_marker == 0)
		return -EINVAL;

	/*
	 * Find completion associated with update_marker requested.
	 * Note: If update completed already, marker will have been
	 * cleared, it won't be found, and function will just return.
	 */

	/* Grab queue lock to protect access to marker list */
	mutex_lock(&fb_data->queue_mutex);

	list_for_each_entry_safe(next_marker, temp,
		&fb_data->full_marker_list, full_list) {
		if (next_marker->update_marker == marker_data->update_marker) {
			if (required_sequence &&
			    next_marker->sleep_sequence != required_sequence) {
				mutex_unlock(&fb_data->queue_mutex);
				return -ESTALE;
			}
			dev_dbg(fb_data->dev, "Waiting for marker %d\n",
				marker_data->update_marker);
			next_marker->waiting = true;
			marker_found = true;
			break;
		}
	}
	if (!marker_found) {
		list_for_each_entry_safe(next_marker, temp,
			&fb_data->sleep_marker_done_list, sleep_done_list) {
			if (next_marker->update_marker !=
			    marker_data->update_marker)
				continue;
			if (required_sequence &&
			    next_marker->sleep_sequence != required_sequence) {
				mutex_unlock(&fb_data->queue_mutex);
				return -ESTALE;
			}
			list_del_init(&next_marker->sleep_done_list);
			marker_data->collision_test =
				next_marker->collision_test;
			ret = next_marker->result;
			completed_marker_found = true;
			break;
		}
	}

	mutex_unlock(&fb_data->queue_mutex);
	if (completed_marker_found) {
		kfree(next_marker);
		return ret;
	}

	/*
	 * Preserve the legacy API's success-on-missing-marker behavior, but make
	 * the sleep barrier fail closed: every tagged marker must have existed and
	 * reached its terminal state.
	 */
	if (!marker_found)
		return require_found ? -ENOENT : ret;

	ret = wait_for_completion_timeout(&next_marker->update_completion,
						timeout);
	if (!ret) {
		dev_err(fb_data->dev,
			"Timed out waiting for update completion\n");
		return -ETIMEDOUT;
	}

	marker_data->collision_test = next_marker->collision_test;
	if (next_marker->result)
		ret = next_marker->result;
	else if (required_sequence)
		/* completion_timeout returns remaining jiffies on success. The
		 * strict sleep barrier must continue to its hardware-idle check. */
		ret = 0;

	mutex_lock(&fb_data->queue_mutex);
	if (next_marker->retained_sleep &&
	    !list_empty(&next_marker->sleep_done_list))
		list_del_init(&next_marker->sleep_done_list);
	mutex_unlock(&fb_data->queue_mutex);

	/* Free update marker object */
	kfree(next_marker);

	return ret;
}

int mxc_epdc_fb_wait_update_complete(struct mxcfb_update_marker_data *marker_data,
						struct fb_info *info)
{
	return epdc_wait_update_marker_timeout(marker_data, info,
						msecs_to_jiffies(5000), false, 0);
}
EXPORT_SYMBOL(mxc_epdc_fb_wait_update_complete);

/* queue_mutex must be held. Restore delayed panel shutdown after a completed
 * strict barrier, or after the user explicitly returns to legacy mode.
 */
static void epdc_sleep_barrier_restore_powerdown_locked(
		struct mxc_epdc_fb_data *fb_data, bool schedule_if_idle)
{
	int delay;
	bool idle;

	if (!fb_data->sleep_barrier_powerdown_held)
		return;
	delay = fb_data->sleep_barrier_saved_pwrdown_delay;
	fb_data->sleep_barrier_powerdown_held = false;
	fb_data->sleep_barrier_pending = false;
	fb_data->pwrdown_delay = delay;
	if (!schedule_if_idle || delay == FB_POWERDOWN_DISABLE ||
	    fb_data->system_suspend_in_progress)
		return;
	idle = list_empty(&fb_data->upd_pending_list) &&
		list_empty(&fb_data->upd_buf_queue) &&
		list_empty(&fb_data->upd_buf_collision_list) &&
		list_empty(&fb_data->full_marker_list) &&
		!fb_data->cur_update && !fb_data->updates_active &&
		is_free_list_full(fb_data);
	if (!idle)
		return;
	mutex_lock(&fb_data->power_mutex);
	if (fb_data->power_state == POWER_STATE_ON) {
		fb_data->powering_down = true;
		schedule_delayed_work(&fb_data->epdc_done_work,
				      msecs_to_jiffies(delay));
	}
	mutex_unlock(&fb_data->power_mutex);
}


static unsigned int epdc_sleep_snapshot_refs_locked(
		struct mxc_epdc_fb_data *fb_data)
{
	unsigned int count = 0;
	u32 i;

	for (i = 0; fb_data->source_snapshots &&
	     i < fb_data->source_snapshot_count; i++)
		if (fb_data->source_snapshots[i].refs)
			count++;
	return count;
}

/* Caller holds queue_mutex. */
static int epdc_hw_software_idle_locked(struct mxc_epdc_fb_data *fb_data)
{
	unsigned int i;

	if (!list_empty(&fb_data->upd_pending_list) ||
	    !list_empty(&fb_data->upd_buf_queue) ||
	    !list_empty(&fb_data->upd_buf_collision_list) ||
	    !list_empty(&fb_data->full_marker_list) || fb_data->cur_update ||
	    fb_data->updates_active || fb_data->waiting_for_idle ||
	    fb_data->waiting_for_wb || fb_data->waiting_for_lut ||
	    fb_data->waiting_for_lut15 || fb_data->cleanup_fence_active ||
	    !is_free_list_full(fb_data) ||
	    epdc_sleep_snapshot_refs_locked(fb_data) ||
	    fb_data->deferred_latest_snapshot.refs ||
	    fb_data->deferred_latest_desc ||
	    work_busy(&fb_data->epdc_intr_work) ||
	    work_busy(&fb_data->epdc_submit_work) ||
	    fb_data->epdc_colliding_luts ||
	    atomic_read(&fb_data->epdc_recovery_state))
		return -EBUSY;
	for (i = 0; i < fb_data->num_luts &&
	     i < ARRAY_SIZE(fb_data->sleep_lut_sequence); i++)
		if (fb_data->sleep_lut_sequence[i])
			return -EBUSY;

	return 0;
}

/* Caller holds queue_mutex -> power_mutex and EPDC clocks are live. */
static int epdc_hw_status_idle_locked(struct mxc_epdc_fb_data *fb_data)
{
	u64 lut_mask, pending_lut_irqs;
	u32 status, irq, irq1, irq2, col, col2;

	status = __raw_readl(EPDC_STATUS);
	lut_mask = (u64)__raw_readl(EPDC_STATUS_LUTS) |
		((u64)__raw_readl(EPDC_STATUS_LUTS2) << 32);
	irq = __raw_readl(EPDC_IRQ);
	irq1 = __raw_readl(EPDC_IRQ1);
	irq2 = __raw_readl(EPDC_IRQ2);
	col = __raw_readl(EPDC_STATUS_COL);
	col2 = __raw_readl(EPDC_STATUS_COL2);
	if (fb_data->rev < 20) {
		pending_lut_irqs = irq & 0xffff;
	} else {
		pending_lut_irqs = (u64)irq1 | ((u64)irq2 << 32);
		if (fb_data->num_luts < 64)
			pending_lut_irqs &= BIT_ULL(fb_data->num_luts) - 1;
	}
	if ((status & EPDC_STATUS_WB_BUSY) || lut_mask || pending_lut_irqs ||
	    (irq & (EPDC_IRQ_WB_CMPLT_IRQ | EPDC_IRQ_LUT_COL_IRQ |
		    EPDC_IRQ_TCE_UNDERRUN_IRQ)) || col || col2)
		return -EBUSY;

	return 0;
}

/* Caller holds queue_mutex -> power_mutex. */
static int epdc_hw_idle_locked(struct mxc_epdc_fb_data *fb_data,
			       bool allow_powering_down)
{
	int ret;

	if (fb_data->powering_down && !allow_powering_down)
		return -EBUSY;
	ret = epdc_hw_software_idle_locked(fb_data);
	if (ret)
		return ret;
	if (fb_data->power_state == POWER_STATE_OFF &&
	    !fb_data->epdc_clocks_enabled && !fb_data->powering_down)
		return 0;
	if (fb_data->power_state != POWER_STATE_ON ||
	    !fb_data->epdc_clocks_enabled)
		return -EHOSTDOWN;

	return epdc_hw_status_idle_locked(fb_data);
}

static int epdc_sleep_barrier_pxp_snapshot(
		struct mxc_epdc_fb_data *fb_data, u32 regs[PXP_PM_TRACE_REGS])
{
	struct dma_chan *chan;
	bool borrowed = false;
	int ret;

	if (!mutex_trylock(&fb_data->pxp_mutex))
		return -EBUSY;
	chan = fb_data->pxp_chan ? &fb_data->pxp_chan->dma_chan : NULL;
	if (!chan) {
		chan = epdc_request_pxp_pm_channel();
		borrowed = !!chan;
	}
	if (!chan) {
		ret = -ENODEV;
	} else {
		ret = pxp_pm_snapshot(chan, regs);
		if (borrowed)
			dma_release_channel(chan);
	}
	mutex_unlock(&fb_data->pxp_mutex);
	return ret;
}

static int epdc_wait_for_hw_idle(struct mxc_epdc_fb_data *fb_data,
				 bool allow_powering_down,
				 unsigned int timeout_ms,
				 u32 *pxp_regs)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	int ret;

	for (;;) {
		unsigned long event, remaining;
		u32 local_pxp_regs[PXP_PM_TRACE_REGS];

		mutex_lock(&fb_data->queue_mutex);
		mutex_lock(&fb_data->power_mutex);
		ret = epdc_hw_idle_locked(fb_data, allow_powering_down);
		mutex_unlock(&fb_data->power_mutex);
		mutex_unlock(&fb_data->queue_mutex);
		if (!ret) {
			ret = epdc_sleep_barrier_pxp_snapshot(fb_data,
							     local_pxp_regs);
			if (!ret) {
				/* Re-read EPDC after PxP reports idle. */
				mutex_lock(&fb_data->queue_mutex);
				mutex_lock(&fb_data->power_mutex);
				ret = epdc_hw_idle_locked(fb_data,
							  allow_powering_down);
				mutex_unlock(&fb_data->power_mutex);
				mutex_unlock(&fb_data->queue_mutex);
				if (!ret) {
					if (pxp_regs)
						memcpy(pxp_regs, local_pxp_regs,
						       sizeof(local_pxp_regs));
				return 0;
				}
			}
		}

		if (ret != -EBUSY)
			return ret;
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;

		event = atomic_read(&fb_data->sleep_barrier_event);
		remaining = deadline - jiffies;
		wait_event_timeout(fb_data->sleep_barrier_waitq,
			atomic_read(&fb_data->sleep_barrier_event) != event,
			min_t(unsigned long, remaining,
			      max_t(unsigned long, 1,
				    msecs_to_jiffies(10))));
	}
}

static int epdc_e70q10_wait_sleep_barrier(
		struct mxc_epdc_fb_data *fb_data,
		const struct mxcfb_e70q10_sleep_barrier *barrier)
{
	unsigned long deadline;
	unsigned int i, j;
	int ret = 0;

	if (!barrier->sequence ||
	    barrier->marker_count > MXCFB_E70Q10_MAX_SLEEP_MARKERS)
		return -EINVAL;
	for (i = 0; i < barrier->marker_count; i++) {
		if (!barrier->markers[i])
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (barrier->markers[i] == barrier->markers[j])
				return -EINVAL;
	}

	ret = mutex_lock_interruptible(&fb_data->pm_control_mutex);
	if (ret)
		return -ERESTARTSYS;

	ret = mutex_lock_interruptible(&hard_lock);
	if (ret) {
		mutex_unlock(&fb_data->pm_control_mutex);
		return -ERESTARTSYS;
	}
	mutex_lock(&fb_data->queue_mutex);
	if (!barrier->marker_count)
		fb_data->last_sleep_sequence = barrier->sequence;
	if (fb_data->system_suspend_in_progress ||
	    fb_data->blank != FB_BLANK_UNBLANK ||
	    fb_data->last_sleep_sequence != barrier->sequence) {
		ret = -EBUSY;
		mutex_unlock(&fb_data->queue_mutex);
		goto out_barrier;
	}
	fb_data->sleep_barrier_waiting = true;
	mutex_unlock(&fb_data->queue_mutex);
	deadline = jiffies + msecs_to_jiffies(E70Q10_SLEEP_BARRIER_TIMEOUT_MS);

	for (i = 0; i < barrier->marker_count; i++) {
		struct mxcfb_update_marker_data marker_data;
		unsigned long remaining;

		if (time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			goto out_barrier;
		}
		remaining = deadline - jiffies;
		marker_data.update_marker = barrier->markers[i];
		marker_data.collision_test = 0;
		ret = epdc_wait_update_marker_timeout(&marker_data, &fb_data->info,
						     remaining, true,
						     barrier->sequence);
		if (ret)
			goto out_barrier;
	}
	mutex_lock(&fb_data->queue_mutex);
	{
		struct update_marker_data *done_marker;

		list_for_each_entry(done_marker,
				    &fb_data->sleep_marker_done_list,
				    sleep_done_list) {
			if (done_marker->sleep_sequence == barrier->sequence) {
				ret = -ENOMSG;
				break;
			}
		}
	}
	mutex_unlock(&fb_data->queue_mutex);
	if (ret)
		goto out_barrier;

	if (time_after_eq(jiffies, deadline)) {
		ret = -ETIMEDOUT;
		goto out_barrier;
	}
	ret = epdc_wait_for_hw_idle(fb_data, false,
			jiffies_to_msecs(deadline - jiffies), NULL);
	if (ret)
		goto out_barrier;

out_barrier:
	mutex_lock(&fb_data->queue_mutex);
	if (!ret)
		fb_data->last_sleep_sequence = barrier->sequence;
	fb_data->sleep_barrier_waiting = false;
	fb_data->sleep_barrier_pending = false;
	if (!ret)
		epdc_sleep_barrier_restore_powerdown_locked(fb_data, true);
	mutex_unlock(&fb_data->queue_mutex);
	mutex_unlock(&hard_lock);
	mutex_unlock(&fb_data->pm_control_mutex);
	return ret;
}

int mxc_epdc_fb_set_pwrdown_delay(u32 pwrdown_delay,
						    struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	mutex_lock(&fb_data->queue_mutex);
	if (fb_data->sleep_barrier_powerdown_held)
		fb_data->sleep_barrier_saved_pwrdown_delay = pwrdown_delay;
	else
		fb_data->pwrdown_delay = pwrdown_delay;
	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_pwrdown_delay);

int mxc_epdc_get_pwrdown_delay(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	int delay;

	mutex_lock(&fb_data->queue_mutex);
	delay = fb_data->sleep_barrier_powerdown_held ?
		fb_data->sleep_barrier_saved_pwrdown_delay :
		fb_data->pwrdown_delay;
	mutex_unlock(&fb_data->queue_mutex);
	return delay;
}
EXPORT_SYMBOL(mxc_epdc_get_pwrdown_delay);

static int mxc_epdc_fb_ioctl(struct fb_info *info, unsigned int cmd,
			     unsigned long arg)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info : g_fb_data;
	void __user *argp = (void __user *)arg;
	int ret = -EINVAL;

	switch (cmd) {
	case MXCFB_SET_WAVEFORM_MODES:
		{
			struct mxcfb_waveform_modes modes;
			if (!copy_from_user(&modes, argp, sizeof(modes))) {
				mxc_epdc_fb_set_waveform_modes(&modes, info);
				ret = 0;
			}
			break;
		}
	case MXCFB_SET_TEMPERATURE:
		{
			int temperature;
			if (!get_user(temperature, (int32_t __user *) arg))
				ret = mxc_epdc_fb_set_temperature(temperature,
					info);
			break;
		}
	case MXCFB_SET_AUTO_UPDATE_MODE:
		{
			u32 auto_mode = 0;
			if (!get_user(auto_mode, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_auto_update(auto_mode,
					info);
			break;
		}
	case MXCFB_SET_UPDATE_SCHEME:
		{
			u32 upd_scheme = 0;
			if (!get_user(upd_scheme, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_upd_scheme(upd_scheme,
					info);
			break;
		}
	case MXCFB_SEND_UPDATE:
		{
			struct mxcfb_update_data upd_data;

			if (mutex_lock_interruptible(&hard_lock) < 0)
				return -ERESTARTSYS;

			if (!copy_from_user(&upd_data, argp,
				sizeof(upd_data))) {
				ret = mxc_epdc_fb_send_update(&upd_data, info);
				if (ret == 0 && copy_to_user(argp, &upd_data,
					sizeof(upd_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}

			mutex_unlock(&hard_lock);

			break;
		}
	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		{
			struct mxcfb_update_marker_data upd_marker_data;
			if (!copy_from_user(&upd_marker_data, argp,
				sizeof(upd_marker_data))) {
				ret = mxc_epdc_fb_wait_update_complete(
					&upd_marker_data, info);
				if (copy_to_user(argp, &upd_marker_data,
					sizeof(upd_marker_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}

			break;
		}

	case MXCFB_SET_PWRDOWN_DELAY:
		{
			int delay = 0;
			if (!get_user(delay, (__u32 __user *) arg))
				ret =
				    mxc_epdc_fb_set_pwrdown_delay(delay, info);
			break;
		}

	case MXCFB_GET_PWRDOWN_DELAY:
		{
			int pwrdown_delay = mxc_epdc_get_pwrdown_delay(info);
			if (put_user(pwrdown_delay,
				(int __user *)argp))
				ret = -EFAULT;
			ret = 0;
			break;
		}

	case MXCFB_GET_WORK_BUFFER:
		{
			/* copy the epdc working buffer to the user space */
			struct mxc_epdc_fb_data *fb_data = info ?
				(struct mxc_epdc_fb_data *)info:g_fb_data;
			flush_cache_all();
			outer_flush_range(fb_data->working_buffer_phys,
				fb_data->working_buffer_phys +
				fb_data->working_buffer_size);
			if (copy_to_user((void __user *)arg,
				(const void *) fb_data->working_buffer_virt,
				fb_data->working_buffer_size))
				ret = -EFAULT;
			else
				ret = 0;
			flush_cache_all();
			outer_flush_range(fb_data->working_buffer_phys,
				fb_data->working_buffer_phys +
				fb_data->working_buffer_size);
			break;
		}

	case MXCFB_DISABLE_EPDC_ACCESS:
		{
			struct mxc_epdc_fb_data *fb_data = info ?
				(struct mxc_epdc_fb_data *)info:g_fb_data;
			mxc_epdc_fb_flush_updates(fb_data);
			/* disable handling any user update request */
			mutex_lock(&hard_lock);
			ret = 0;
			break;
		}

	case MXCFB_ENABLE_EPDC_ACCESS:
		{
			/* enable user update handling again */
			mutex_unlock(&hard_lock);
			ret = 0;
			break;
		}
	case MXCFB_E70Q10_WAIT_SLEEP_BARRIER:
		{
			struct mxcfb_e70q10_sleep_barrier barrier;
			if (copy_from_user(&barrier, argp, sizeof(barrier)))
				ret = -EFAULT;
			else
				ret = epdc_e70q10_wait_sleep_barrier(fb_data,
								     &barrier);
			break;
		}

	default:
		break;
	}
	return ret;
}

#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
static void mxc_epdc_fb_update_pages(struct mxc_epdc_fb_data *fb_data,
				     u16 y1, u16 y2)
{
	struct mxcfb_update_data update;

	/* Do partial screen update, Update full horizontal lines */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = y1;
	update.update_region.height = y2 - y1;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_mode = UPDATE_MODE_FULL;
	update.update_marker = 0;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, &fb_data->info);
}

/* this is called back from the deferred io workqueue */
static void mxc_epdc_fb_deferred_io(struct fb_info *info,
				    struct list_head *pagelist)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct page *page;
	unsigned long beg, end;
	int y1, y2, miny, maxy;

	if (fb_data->auto_mode != AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		return;

	miny = INT_MAX;
	maxy = 0;
	list_for_each_entry(page, pagelist, lru) {
		beg = page->index << PAGE_SHIFT;
		end = beg + PAGE_SIZE - 1;
		y1 = beg / info->fix.line_length;
		y2 = end / info->fix.line_length;
		if (y2 >= fb_data->epdc_fb_var.yres)
			y2 = fb_data->epdc_fb_var.yres - 1;
		if (miny > y1)
			miny = y1;
		if (maxy < y2)
			maxy = y2;
	}

	mxc_epdc_fb_update_pages(fb_data, miny, maxy);
}
#endif

static int mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data)
{
	int ret;

	if (fb_data->in_init)
		return 0;

	/* Grab queue lock to prevent any new updates from being submitted */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * 3 places to check for updates that are active or pending:
	 *   1) Updates in the pending list
	 *   2) Update buffers in use (e.g., PxP processing)
	 *   3) Active updates to panel - We can key off of EPDC
	 *      power state to know if we have active updates.
	 */
	if (!list_empty(&fb_data->upd_pending_list) ||
		!is_free_list_full(fb_data) ||
		(fb_data->updates_active == true)) {
		/* Initialize event signalling updates are done */
		init_completion(&fb_data->updates_done);
		fb_data->waiting_for_idle = true;

		mutex_unlock(&fb_data->queue_mutex);
		/* Wait for any currently active updates to complete */
		ret = wait_for_completion_timeout(&fb_data->updates_done,
						msecs_to_jiffies(8000));
		if (!ret) {
			dev_err(fb_data->dev,
				"Flush updates timeout! ret = 0x%x\n", ret);
			mutex_lock(&fb_data->queue_mutex);
			fb_data->waiting_for_idle = false;
			mutex_unlock(&fb_data->queue_mutex);
			epdc_recovery_dump(fb_data, "flush-timeout");
			return -ETIMEDOUT;
		}

		mutex_lock(&fb_data->queue_mutex);
		fb_data->waiting_for_idle = false;
	}

	mutex_unlock(&fb_data->queue_mutex);
	return 0;
}


static int mxc_epdc_fb_blank(int blank, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int old_blank = fb_data->blank;
	int pwrdown_delay;
	unsigned long timeout;
	int ret = 0;
	bool powered_on;

	dev_dbg(fb_data->dev, "blank = %d\n", blank);
	if (old_blank == blank && blank != FB_BLANK_POWERDOWN)
		return 0;
	if (old_blank == blank && blank == FB_BLANK_POWERDOWN) {
		mutex_lock(&fb_data->power_mutex);
		powered_on = fb_data->power_state == POWER_STATE_ON;
		mutex_unlock(&fb_data->power_mutex);
		if (!powered_on)
			return 0;
	}

	fb_data->blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
		ret = mxc_epdc_fb_flush_updates(fb_data);
		if (ret)
			break;

		mutex_lock(&fb_data->power_mutex);
		powered_on = fb_data->power_state == POWER_STATE_ON;
		pwrdown_delay = fb_data->pwrdown_delay;
		if (powered_on && pwrdown_delay == FB_POWERDOWN_DISABLE) {
			fb_data->powering_down = true;
			mutex_unlock(&fb_data->power_mutex);
			ret = epdc_runtime_powerdown(fb_data);
			break;
		}
		if (powered_on) {
			fb_data->powering_down = true;
			fb_data->wait_for_powerdown = true;
			init_completion(&fb_data->powerdown_compl);
			schedule_delayed_work(&fb_data->epdc_done_work,
				msecs_to_jiffies(max_t(int, pwrdown_delay, 0)));
			mutex_unlock(&fb_data->power_mutex);
			timeout = msecs_to_jiffies(max_t(int,
				7000, pwrdown_delay + 7000));
			if (!wait_for_completion_timeout(&fb_data->powerdown_compl,
						 timeout)) {
				cancel_delayed_work_sync(&fb_data->epdc_done_work);
				mutex_lock(&fb_data->power_mutex);
				fb_data->wait_for_powerdown = false;
				if (fb_data->power_state == POWER_STATE_ON)
					fb_data->powering_down = false;
				mutex_unlock(&fb_data->power_mutex);
				ret = -ETIMEDOUT;
				break;
			}
			mutex_lock(&fb_data->power_mutex);
			powered_on = fb_data->power_state == POWER_STATE_ON;
			mutex_unlock(&fb_data->power_mutex);
			if (powered_on)
				ret = -EBUSY;
		} else {
			mutex_unlock(&fb_data->power_mutex);
		}
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		mxc_epdc_fb_flush_updates(fb_data);
		break;
	}

	if (ret)
		fb_data->blank = old_blank;
	return ret;
}

static void mxc_epdc_fb_clear_row_padding(struct mxc_epdc_fb_data *fb_data,
					  u32 fb_offset)
{
	u32 y;
	u32 bytes_per_pixel = fb_data->epdc_fb_var.bits_per_pixel / 8;
	u32 visible_bytes = fb_data->epdc_fb_var.xres * bytes_per_pixel;
	u32 stride = fb_data->info.fix.line_length;
	u8 *page;

	if (!of_machine_is_compatible("ridi,e70q10") || !bytes_per_pixel ||
	    visible_bytes >= stride)
		return;

	page = (u8 *)fb_data->info.screen_base + fb_offset;
	for (y = 0; y < fb_data->epdc_fb_var.yres; y++)
		memset(page + y * stride + visible_bytes, 0xff,
		       stride - visible_bytes);
}

static int mxc_epdc_fb_pan_display(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	u_int y_bottom;

	dev_dbg(info->device, "%s: var->yoffset %d, info->var.yoffset %d\n",
		 __func__, var->yoffset, info->var.yoffset);
	/* check if var is valid; also, xpan is not supported */
	if (!var || (var->xoffset != info->var.xoffset) ||
	    (var->yoffset + var->yres > var->yres_virtual)) {
		dev_dbg(info->device, "x panning not supported\n");
		return -EINVAL;
	}

	y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	if (y_bottom > info->var.yres_virtual)
		return -EINVAL;

	mutex_lock(&fb_data->queue_mutex);

	fb_data->fb_offset = (var->yoffset * var->xres_virtual + var->xoffset)
		* (var->bits_per_pixel) / 8;
	fb_data->fb_source_generation++;
	if (fb_data->page_lifetime_diag) {
		unsigned long page_flags;
		int page = epdc_fb_page_index(fb_data, fb_data->fb_offset);

		spin_lock_irqsave(&fb_data->fb_page_lock, page_flags);
		if (page >= 0)
			fb_data->fb_page_generations[page]++;
		else
			fb_data->untracked_fb_pan++;
		spin_unlock_irqrestore(&fb_data->fb_page_lock, page_flags);
	}
	/* Apply the E70Q10 portrait stride workaround before panning. */
	mxc_epdc_fb_clear_row_padding(fb_data, fb_data->fb_offset);

	fb_data->epdc_fb_var.xoffset = var->xoffset;
	fb_data->epdc_fb_var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}

/*
 * E70Q10 does not draw a framebuffer software cursor.  Returning success
 * prevents fbcon's soft_cursor() fallback for this machine.  Other boards
 * return an error so fbcon keeps its existing soft_cursor() behavior.
 */
static int e70q10_mxc_fb_cursor(struct fb_info *info,
				struct fb_cursor *cursor)
{
	if (of_machine_is_compatible("ridi,e70q10"))
		return 0;

	return -ENODEV;
}

static struct fb_ops mxc_epdc_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxc_epdc_fb_check_var,
	.fb_set_par = mxc_epdc_fb_set_par,
	.fb_setcmap = mxc_epdc_fb_setcmap,
	.fb_setcolreg = mxc_epdc_fb_setcolreg,
	.fb_pan_display = mxc_epdc_fb_pan_display,
	.fb_ioctl = mxc_epdc_fb_ioctl,
	.fb_mmap = mxc_epdc_fb_mmap,
	.fb_blank = mxc_epdc_fb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = e70q10_mxc_fb_cursor,
};

#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
static struct fb_deferred_io mxc_epdc_fb_defio = {
	.delay = HZ,
	.deferred_io = mxc_epdc_fb_deferred_io,
};
#endif

static void epdc_done_work_func(struct work_struct *work)
{
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data,
			epdc_done_work.work);
	bool boot_initial_powerdown;

	mutex_lock(&fb_data->power_mutex);
	boot_initial_powerdown = fb_data->boot_initial_powerdown_pending;
	fb_data->boot_initial_powerdown_pending = false;
	mutex_unlock(&fb_data->power_mutex);

	if (boot_initial_powerdown) {
		epdc_powerdown(fb_data);
		return;
	}

	epdc_runtime_powerdown(fb_data);
}

static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data)
{
	int count = 0;
	struct update_data_list *plist;

	/* Count buffers in free buffer list */
	list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
		count++;

	/* Check to see if all buffers are in this list */
	if (count == fb_data->max_num_updates)
		return true;
	else
		return false;
}

static irqreturn_t mxc_epdc_irq_handler(int irq, void *dev_id)
{
	struct mxc_epdc_fb_data *fb_data = dev_id;
	u32 ints_fired, luts1_ints_fired, luts2_ints_fired;


	/*
	 * If we just completed one-time panel init, bypass
	 * queue handling, clear interrupt and return
	 */
	if (fb_data->in_init) {
		if (epdc_is_working_buffer_complete()) {
			epdc_working_buf_intr(false);
			epdc_clear_working_buf_irq();
			dev_dbg(fb_data->dev, "Cleared WB for init update\n");
		}

		if (epdc_is_lut_complete(fb_data->rev, 0)) {
			epdc_lut_complete_intr(fb_data->rev, 0, false);
			epdc_clear_lut_complete_irq(fb_data->rev, 0);
			fb_data->in_init = false;
			dev_dbg(fb_data->dev, "Cleared LUT complete for init update\n");
		}

		return IRQ_HANDLED;
	}

	ints_fired = __raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ);
	if (fb_data->rev < 20) {
		luts1_ints_fired = 0;
		luts2_ints_fired = 0;
	} else {
		luts1_ints_fired = __raw_readl(EPDC_IRQ_MASK1) & __raw_readl(EPDC_IRQ1);
		luts2_ints_fired = __raw_readl(EPDC_IRQ_MASK2) & __raw_readl(EPDC_IRQ2);
	}

	if (!(ints_fired || luts1_ints_fired || luts2_ints_fired))
		return IRQ_HANDLED;

	if (__raw_readl(EPDC_IRQ) & EPDC_IRQ_TCE_UNDERRUN_IRQ) {
		/* Acknowledge in hard IRQ; reset and queue repair run in work context. */
		__raw_writel(EPDC_IRQ_TCE_UNDERRUN_IRQ, EPDC_IRQ_CLEAR);
		atomic_inc(&fb_data->tce_underrun_count);
		/* The manual staged experiment owns reset until it returns. */
		epdc_request_recovery(fb_data);
		/* Recovery owns all outstanding queue and LUT reconciliation. */
		__raw_writel(ints_fired, EPDC_IRQ_MASK_CLEAR);
		__raw_writel(luts1_ints_fired, EPDC_IRQ_MASK1_CLEAR);
		__raw_writel(luts2_ints_fired, EPDC_IRQ_MASK2_CLEAR);
		return IRQ_HANDLED;
	}

	/* Check if we are waiting on EOF to sync a new update submission */
	if (epdc_signal_eof()) {
		epdc_eof_intr(false);
		epdc_clear_eof_irq();
		complete(&fb_data->eof_event);
	}

	/*
	 * Workaround for EPDC v2.0/v2.1 errata: Must read collision status
	 * before clearing IRQ, or else collision status for bits 16:63
	 * will be automatically cleared.  So we read it here, and there is
	 * no conflict with using it in epdc_intr_work_func since the
	 * working buffer processing flow is strictly sequential (i.e.,
	 * only one WB processing done at a time, so the data grabbed
	 * here should be up-to-date and accurate when the WB processing
	 * completes.  Also, note that there is no impact to other versions
	 * of EPDC by reading LUT status here.
	 */
	if (fb_data->cur_update != NULL)
		fb_data->epdc_colliding_luts = epdc_get_colliding_luts(fb_data->rev);

	/* Clear the interrupt mask for any interrupts signalled */
	__raw_writel(ints_fired, EPDC_IRQ_MASK_CLEAR);
	__raw_writel(luts1_ints_fired, EPDC_IRQ_MASK1_CLEAR);
	__raw_writel(luts2_ints_fired, EPDC_IRQ_MASK2_CLEAR);

	dev_dbg(fb_data->dev, "EPDC interrupts fired = 0x%x, "
		"LUTS1 fired = 0x%x, LUTS2 fired = 0x%x\n",
		ints_fired, luts1_ints_fired, luts2_ints_fired);

	queue_work(fb_data->epdc_intr_workqueue,
		&fb_data->epdc_intr_work);

	return IRQ_HANDLED;
}

static void epdc_intr_work_func(struct work_struct *work)
{
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_intr_work);
	struct update_data_list *collision_update;
	struct mxcfb_rect *next_upd_region;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	int temp_index;
	u64 temp_mask;
	u32 lut;
	bool ignore_collision = false;
	int i;
	bool wb_lut_done = false;
	bool free_update = true;
	int next_lut, epdc_next_lut_15;
	u32 epdc_luts_active, epdc_wb_busy, epdc_luts_avail, epdc_lut_cancelled;
	u32 epdc_collision;
	u64 epdc_irq_stat;
	u64 collision_luts_snapshot;
	bool epdc_waiting_on_wb;
	bool snapshot_collision_retry = false;
	u32 coll_coord, coll_size;
	struct mxcfb_rect coll_region;

	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);

	/* Capture EPDC status one time to limit exposure to race conditions */
	epdc_luts_active = epdc_any_luts_active(fb_data->rev);
	epdc_wb_busy = epdc_is_working_buffer_busy();
	epdc_lut_cancelled = epdc_is_lut_cancelled();
	epdc_luts_avail = epdc_any_luts_available();
	epdc_collision = epdc_is_collision();
	collision_luts_snapshot = fb_data->epdc_colliding_luts;
	if (fb_data->rev < 20)
		epdc_irq_stat = __raw_readl(EPDC_IRQ);
	else
		epdc_irq_stat = (u64)__raw_readl(EPDC_IRQ1) |
			((u64)__raw_readl(EPDC_IRQ2) << 32);
	epdc_waiting_on_wb = (fb_data->cur_update != NULL) ? true : false;
	/*
	 * Preserve collision ordering before the LUT-completion loop clears
	 * lut_update_order[].  A collided LUT may complete in this same IRQ work
	 * batch, or in an earlier batch while this descriptor's WB is still live.
	 */
	if (of_machine_is_compatible("ridi,e70q10") &&
	    epdc_waiting_on_wb && epdc_collision &&
	    fb_data->cur_update->update_desc) {
		struct update_desc_list *desc =
			fb_data->cur_update->update_desc;

		fb_data->cur_update->collision_mask |= collision_luts_snapshot;
		for (i = 0; i < fb_data->num_luts; i++) {
			u32 collided_order;

			if (!(collision_luts_snapshot & BIT_ULL(i)))
				continue;
			collided_order = fb_data->lut_update_order[i];
			if (collided_order > desc->update_order &&
			    (!desc->collided_newer_seen ||
			     collided_order > desc->collided_newer_order)) {
				desc->collided_newer_order = collided_order;
				desc->collided_newer_seen = true;
			}
		}
	}

	/* Free any LUTs that have completed */
	for (i = 0; i < fb_data->num_luts; i++) {
		if ((epdc_irq_stat & (1ULL << i)) == 0)
			continue;
		if (fb_data->sleep_lut_sequence[i]) {
			fb_data->sleep_lut_sequence[i] = 0;
		}
		/* Preserve completion evidence across separate IRQ work batches. */
		if (epdc_waiting_on_wb && fb_data->cur_update &&
		    fb_data->cur_update->update_desc &&
		    (fb_data->cur_update->collision_mask & BIT_ULL(i)) &&
		    fb_data->lut_update_order[i] >
			fb_data->cur_update->update_desc->update_order)
			fb_data->cur_update->update_desc->
				collided_newer_completed_before_wb = true;

		dev_dbg(fb_data->dev, "LUT %d completed\n", i);

		/* Disable IRQ for completed LUT */
		epdc_lut_complete_intr(fb_data->rev, i, false);

		/*
		 * Go through all updates in the collision list and
		 * unmask any updates that were colliding with
		 * the completed LUT.
		 */
		list_for_each_entry(collision_update,
				    &fb_data->upd_buf_collision_list, list) {
			collision_update->collision_mask =
			    collision_update->collision_mask & ~BIT_ULL(i);
		}

		epdc_clear_lut_complete_irq(fb_data->rev, i);

		fb_data->luts_complete_wb |= 1ULL << i;

		fb_data->lut_update_order[i] = 0;

		/* Signal completion if submit workqueue needs a LUT */
		if (fb_data->waiting_for_lut) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_lut = false;
		}

		/* Signal completion if LUT15 free and is needed */
		if (fb_data->waiting_for_lut15 && (i == 15)) {
			complete(&fb_data->lut15_free);
			fb_data->waiting_for_lut15 = false;
		}

		/* Detect race condition where WB and its LUT complete
		   (i.e. full update completes) in one swoop */
		if (epdc_waiting_on_wb &&
			(i == fb_data->cur_update->lut_num))
			wb_lut_done = true;

		if (fb_data->cleanup_fence_active &&
		    i == fb_data->cleanup_fence_lut &&
		    !(epdc_waiting_on_wb && epdc_collision &&
		      epdc_is_cleanup_desc(fb_data,
			fb_data->cur_update->update_desc) &&
		      i == fb_data->cur_update->lut_num)) {
			fb_data->cleanup_fence_active = false;
			fb_data->cleanup_fence_lut = INVALID_LUT;
		}

		/* Signal completion if anyone waiting on this LUT */
		if (!wb_lut_done)
			list_for_each_entry_safe(next_marker, temp,
				&fb_data->full_marker_list,
				full_list) {
				if (next_marker->lut_num != i)
					continue;

				/* Signal completion of update */
				dev_dbg(fb_data->dev, "Signaling marker %d\n",
					next_marker->update_marker);
				epdc_complete_marker_locked(fb_data, next_marker, 0);
			}
	}

	/* Check to see if all updates have completed */
	if (list_empty(&fb_data->upd_pending_list) &&
		is_free_list_full(fb_data) &&
		!epdc_waiting_on_wb &&
		!epdc_luts_active) {

		fb_data->updates_active = false;

		if (fb_data->pwrdown_delay != FB_POWERDOWN_DISABLE) {
			/*
			 * Set variable to prevent overlapping
			 * enable/disable requests
			 */
			fb_data->powering_down = true;

			/* Schedule task to disable EPDC HW until next update */
			schedule_delayed_work(&fb_data->epdc_done_work,
				msecs_to_jiffies(fb_data->pwrdown_delay));

			/* Reset counter to reduce chance of overflow */
			fb_data->order_cnt = 0;
		}

		if (fb_data->waiting_for_idle)
			complete(&fb_data->updates_done);
	}

	/* Is Working Buffer busy? */
	if (epdc_wb_busy) {
		/* Can't submit another update until WB is done */
		mutex_unlock(&fb_data->queue_mutex);
		epdc_sleep_barrier_wake(fb_data);
		return;
	}

	/*
	 * Were we waiting on working buffer?
	 * If so, update queues and check for collisions
	 */
	if (epdc_waiting_on_wb) {
		dev_dbg(fb_data->dev, "\nWorking buffer completed\n");

		/* Signal completion if submit workqueue was waiting on WB */
		if (fb_data->waiting_for_wb) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_wb = false;
		}

		if (fb_data->cur_update->update_desc->upd_data.flags
			& EPDC_FLAG_TEST_COLLISION) {
			/* This was a dry run to test for collision */

			/* Signal marker */
			list_for_each_entry_safe(next_marker, temp,
				&fb_data->full_marker_list,
				full_list) {
				if (next_marker->lut_num != DRY_RUN_NO_LUT)
					continue;

				if (epdc_collision)
					next_marker->collision_test = true;
				else
					next_marker->collision_test = false;

				dev_dbg(fb_data->dev,
					"In IRQ, collision_test = %d\n",
					next_marker->collision_test);

				/* Signal completion of update */
				dev_dbg(fb_data->dev, "Signaling marker "
					"for dry-run - %d\n",
					next_marker->update_marker);
				epdc_complete_marker_locked(fb_data, next_marker, 0);
			}
		} else if (epdc_lut_cancelled && !epdc_collision) {
			/*
			* Note: The update may be cancelled (void) if all
			* pixels collided. In that case we handle it as a
			* collision, not a cancel.
			*/
			if (fb_data->sleep_lut_sequence[
				fb_data->cur_update->lut_num]) {
				u32 lut = fb_data->cur_update->lut_num;

				fb_data->sleep_lut_sequence[lut] = 0;
			}

			/* Clear LUT status (might be set if no AUTOWV used) */
			if (fb_data->cur_update->update_desc->deferred_retry)
				fb_data->deferred_latest_retry_dropped_error++;

			/*
			 * Disable and clear IRQ for the LUT used.
			 * Even though LUT is cancelled in HW, the LUT
			 * complete bit may be set if AUTOWV not used.
			 */
			epdc_lut_complete_intr(fb_data->rev,
					fb_data->cur_update->lut_num, false);
			epdc_clear_lut_complete_irq(fb_data->rev,
					fb_data->cur_update->lut_num);

			fb_data->lut_update_order[fb_data->cur_update->lut_num] = 0;
			if (epdc_is_cleanup_desc(fb_data,
					fb_data->cur_update->update_desc)) {
				fb_data->cleanup_fence_active = false;
				fb_data->cleanup_fence_lut = INVALID_LUT;
			}

			/* Signal completion if submit workqueue needs a LUT */
			if (fb_data->waiting_for_lut) {
				complete(&fb_data->update_res_free);
				fb_data->waiting_for_lut = false;
			}

			list_for_each_entry_safe(next_marker, temp,
				&fb_data->cur_update->update_desc->upd_marker_list,
				upd_list) {

				/* Signal completion of update */
				dev_dbg(fb_data->dev,
					"Signaling marker (cancelled) %d\n",
					next_marker->update_marker);
				epdc_complete_marker_locked(fb_data, next_marker,
							   -ECANCELED);
			}
		} else if (epdc_collision) {
			/* Real update (no dry-run), collision occurred */
			bool cleanup_collision = epdc_is_cleanup_desc(fb_data,
				fb_data->cur_update->update_desc);
			bool ordinary_collision =
				epdc_is_ordinary_collision_update(fb_data,
					fb_data->cur_update->update_desc);
			/*
			 * A collided full cleanup may already be driving its
			 * non-colliding pixels on the LUT assigned to this attempt.
			 * Waiting only for the opposing LUTs resubmits the full rectangle
			 * against its own still-active LUT and creates a livelock.  Include
			 * this attempt's LUT in the fence before deciding it is retryable.
			 */
			if (cleanup_collision && fb_data->cur_update->lut_num >= 0)
				fb_data->cur_update->collision_mask |=
					BIT_ULL(fb_data->cur_update->lut_num);

			/* Clear collisions that completed since WB began */
			fb_data->cur_update->collision_mask &=
				~fb_data->luts_complete_wb;

			dev_dbg(fb_data->dev, "Collision mask = 0x%llx\n",
			       fb_data->epdc_colliding_luts);
			/* For EPDC 2.0 and later, minimum collision bounds
			   are provided by HW.  Recompute new bounds here. */
			if ((fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT)
				&& (fb_data->rev >= 20) && !cleanup_collision) {
				u32 xres, yres, rotate;
				struct mxcfb_rect *cur_upd_rect =
					&fb_data->cur_update->update_desc->upd_data.update_region;

				/* Get collision region coords from EPDC */
				coll_coord = __raw_readl(EPDC_UPD_COL_CORD);
				coll_size = __raw_readl(EPDC_UPD_COL_SIZE);
				coll_region.left =
					(coll_coord & EPDC_UPD_COL_CORD_XCORD_MASK)
						>> EPDC_UPD_COL_CORD_XCORD_OFFSET;
				coll_region.top =
					(coll_coord & EPDC_UPD_COL_CORD_YCORD_MASK)
						>> EPDC_UPD_COL_CORD_YCORD_OFFSET;
				coll_region.width =
					(coll_size & EPDC_UPD_COL_SIZE_WIDTH_MASK)
						>> EPDC_UPD_COL_SIZE_WIDTH_OFFSET;
				coll_region.height =
					(coll_size & EPDC_UPD_COL_SIZE_HEIGHT_MASK)
						>> EPDC_UPD_COL_SIZE_HEIGHT_OFFSET;
				dev_dbg(fb_data->dev, "Coll region: l = %d, "
					"t = %d, w = %d, h = %d\n",
					coll_region.left, coll_region.top,
					coll_region.width, coll_region.height);


				/* Convert coords back to orig orientation */
				switch (fb_data->epdc_fb_var.rotate) {
				case FB_ROTATE_CW:
					xres = fb_data->epdc_fb_var.yres;
					yres = fb_data->epdc_fb_var.xres;
					rotate = FB_ROTATE_CCW;
					break;
				case FB_ROTATE_UD:
					xres = fb_data->epdc_fb_var.xres;
					yres = fb_data->epdc_fb_var.yres;
					rotate = FB_ROTATE_UD;
					break;
				case FB_ROTATE_CCW:
					xres = fb_data->epdc_fb_var.yres;
					yres = fb_data->epdc_fb_var.xres;
					rotate = FB_ROTATE_CW;
					break;
				default:
					xres = fb_data->epdc_fb_var.xres;
					yres = fb_data->epdc_fb_var.yres;
					rotate = FB_ROTATE_UR;
					break;
				}
				adjust_coordinates(xres, yres, rotate,
						&coll_region, cur_upd_rect);

				dev_dbg(fb_data->dev, "Adj coll region: l = %d, "
					"t = %d, w = %d, h = %d\n",
					cur_upd_rect->left, cur_upd_rect->top,
					cur_upd_rect->width,
					cur_upd_rect->height);
			}

			/*
			 * If we collide with newer updates, then
			 * we don't need to re-submit the update. The
			 * idea is that the newer updates should take
			 * precedence anyways, so we don't want to
			 * overwrite them.
			 */
			/*
			 * A cleanup is a barrier and must survive even if an unusual
			 * producer submits a newer LUT while it is active. Ordinary
			 * updates retain the existing newer-update-wins behaviour.
			 */
			if (!cleanup_collision) {
				if (ordinary_collision &&
				    fb_data->cur_update->update_desc->collided_newer_seen) {
					ignore_collision = true;
					fb_data->collision_retry_dropped_newer++;
					if (fb_data->cur_update->update_desc->
						collided_newer_completed_before_wb)
						fb_data->collision_newer_completed_before_wb++;
				}
				for (temp_mask = fb_data->cur_update->collision_mask,
				     lut = 0; temp_mask != 0;
				     lut++, temp_mask = temp_mask >> 1) {
					if (ignore_collision)
						break;
					if (!(temp_mask & 0x1))
						continue;

					if (fb_data->lut_update_order[lut] >=
					    fb_data->cur_update->update_desc->update_order) {
						dev_dbg(fb_data->dev,
							"Ignoring collision with"
							"newer update.\n");
						ignore_collision = true;
						fb_data->collision_retry_dropped_newer++;
						break;
					}
				}
			}

			if (!ignore_collision) {
				free_update = false;
				/*
				 * If update has markers, clear the LUTs to
				 * avoid signalling that they have completed.
				 */
				list_for_each_entry_safe(next_marker, temp,
					&fb_data->cur_update->update_desc->upd_marker_list,
					upd_list)
					next_marker->lut_num = INVALID_LUT;

				/* Move to collision list */
				list_add_tail(&fb_data->cur_update->list,
					 &fb_data->upd_buf_collision_list);
				fb_data->collision_created++;
				{
					u64 count = 0;
					struct update_data_list *entry;

					list_for_each_entry(entry,
						&fb_data->upd_buf_collision_list, list)
						count++;
					fb_data->collision_list_peak = max(
						fb_data->collision_list_peak, count);
				}
			}
		}

		/* Do we need to free the current update descriptor? */
		if (free_update) {
			if (!epdc_collision && !epdc_lut_cancelled &&
			    fb_data->cur_update->update_desc->deferred_retry)
				fb_data->deferred_latest_retry_success++;
			if (!epdc_collision && !epdc_lut_cancelled &&
			    fb_data->cur_update->update_desc->fb_source_generation >
				fb_data->latest_completed_generation)
				fb_data->latest_completed_generation =
				fb_data->cur_update->update_desc->fb_source_generation;
			/* Handle condition where WB & LUT are both complete */
			if (wb_lut_done)
				list_for_each_entry_safe(next_marker, temp,
					&fb_data->cur_update->update_desc->upd_marker_list,
					upd_list) {

					/* Signal completion of update */
					dev_dbg(fb_data->dev,
						"Signaling marker (wb) %d\n",
						next_marker->update_marker);
					epdc_complete_marker_locked(fb_data,
						next_marker, 0);
				}
			/* Free marker list and update descriptor */
			epdc_free_update_desc_locked(fb_data, fb_data->cur_update->update_desc);

			/* Add to free buffer list */
			list_add_tail(&fb_data->cur_update->list,
				 &fb_data->upd_buf_free_list);

			/* Check to see if all updates have completed */
			if (list_empty(&fb_data->upd_pending_list) &&
				is_free_list_full(fb_data) &&
				!epdc_luts_active) {

				fb_data->updates_active = false;

				if (fb_data->pwrdown_delay !=
						FB_POWERDOWN_DISABLE) {
					/*
					 * Set variable to prevent overlapping
					 * enable/disable requests
					 */
					fb_data->powering_down = true;

					/* Schedule EPDC disable */
					schedule_delayed_work(&fb_data->epdc_done_work,
						msecs_to_jiffies(fb_data->pwrdown_delay));

					/* Reset counter to reduce chance of overflow */
					fb_data->order_cnt = 0;
				}

				if (fb_data->waiting_for_idle)
					complete(&fb_data->updates_done);
			}
		}

		/* Clear current update */
		fb_data->cur_update = NULL;

		/* Clear IRQ for working buffer */
		epdc_working_buf_intr(false);
		epdc_clear_working_buf_irq();
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		/* Schedule task to submit collision and pending update */
		if (!fb_data->powering_down)
			queue_work(fb_data->epdc_submit_workqueue,
				&fb_data->epdc_submit_work);

		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);
		epdc_sleep_barrier_wake(fb_data);

		return;
	}

	/* Snapshot update scheme processing */

	/* Check to see if any LUTs are free */
	if (!epdc_luts_avail) {
		dev_dbg(fb_data->dev, "No luts available.\n");
		mutex_unlock(&fb_data->queue_mutex);
		epdc_sleep_barrier_wake(fb_data);
		return;
	}

	epdc_next_lut_15 = epdc_choose_next_lut(fb_data->rev, &next_lut);
	/* Check to see if there is a valid LUT to use */
	if (epdc_next_lut_15 && fb_data->tce_prevent && (fb_data->rev < 20)) {
		dev_dbg(fb_data->dev, "Must wait for LUT15\n");
		mutex_unlock(&fb_data->queue_mutex);
		epdc_sleep_barrier_wake(fb_data);
		return;
	}

	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */
	list_for_each_entry(collision_update,
			    &fb_data->upd_buf_collision_list, list) {

		if (collision_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");
		/*
		 * We have a collision cleared, so select it
		 * and we will retry the update
		 */
		fb_data->cur_update = collision_update;
		snapshot_collision_retry = true;
		list_del_init(&fb_data->cur_update->list);
		break;
	}

	/*
	 * If we didn't find a collision update ready to go,
	 * we try to grab one from the update queue
	 */
	if (fb_data->cur_update == NULL) {
		/* Is update list empty? */
		if (list_empty(&fb_data->upd_buf_queue)) {
			dev_dbg(fb_data->dev, "No pending updates.\n");

			/* No updates pending, so we are done */
			mutex_unlock(&fb_data->queue_mutex);
			epdc_sleep_barrier_wake(fb_data);
			return;
		} else {
			dev_dbg(fb_data->dev, "Found a pending update!\n");

			/* Process next item in update list */
			fb_data->cur_update =
			    list_entry(fb_data->upd_buf_queue.next,
				       struct update_data_list, list);
			list_del_init(&fb_data->cur_update->list);
		}
	}

	/* Use LUT selected above */
	fb_data->cur_update->lut_num = next_lut;
	if (epdc_is_cleanup_desc(fb_data,
			fb_data->cur_update->update_desc)) {
		fb_data->cur_update->update_desc->collision_resubmit |=
			snapshot_collision_retry;
		fb_data->cleanup_fence_lut = next_lut;
	}

	/* Associate LUT with update markers */
	list_for_each_entry_safe(next_marker, temp,
		&fb_data->cur_update->update_desc->upd_marker_list, upd_list)
		next_marker->lut_num = fb_data->cur_update->lut_num;
	if (fb_data->cur_update->update_desc->sleep_sequence) {
		fb_data->sleep_lut_sequence[fb_data->cur_update->lut_num] =
			fb_data->cur_update->update_desc->sleep_sequence;
	}

	/* Mark LUT as containing new update */
	fb_data->lut_update_order[fb_data->cur_update->lut_num] =
		fb_data->cur_update->update_desc->update_order;

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->rev, fb_data->cur_update->lut_num, true);

	/* Program EPDC update to process buffer */
	next_upd_region =
		&fb_data->cur_update->update_desc->upd_data.update_region;
	if (fb_data->cur_update->update_desc->upd_data.temp
		!= TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			fb_data->cur_update->update_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	epdc_set_update_addr(fb_data->cur_update->phys_addr +
				fb_data->cur_update->update_desc->epdc_offs);
	epdc_set_update_coord(next_upd_region->left, next_upd_region->top);
	epdc_set_update_dimensions(next_upd_region->width,
				   next_upd_region->height);
	if (fb_data->rev > 20)
		epdc_set_update_stride(fb_data->cur_update->update_desc->epdc_stride);
	if (fb_data->wv_modes_update &&
		(fb_data->cur_update->update_desc->upd_data.waveform_mode
			== WAVEFORM_MODE_AUTO)) {
		epdc_set_update_waveform(&fb_data->wv_modes);
		fb_data->wv_modes_update = false;
	}

	epdc_submit_update(fb_data, fb_data->cur_update->lut_num,
			   fb_data->cur_update->update_desc->upd_data.waveform_mode,
			   fb_data->cur_update->update_desc->upd_data.update_mode,
			   false, false, 0);

	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);
	epdc_sleep_barrier_wake(fb_data);

	return;
}

static void draw_mode0(struct mxc_epdc_fb_data *fb_data)
{
	u32 *upd_buf_ptr;
	int i;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 xres, yres;

	upd_buf_ptr = (u32 *)fb_data->info.screen_base;

	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->rev, 0, true);

	/* Use unrotated (native) width/height */
	if ((screeninfo->rotate == FB_ROTATE_CW) ||
		(screeninfo->rotate == FB_ROTATE_CCW)) {
		xres = screeninfo->yres;
		yres = screeninfo->xres;
	} else {
		xres = screeninfo->xres;
		yres = screeninfo->yres;
	}

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(fb_data->phys_start);
	epdc_set_update_coord(0, 0);
	epdc_set_update_dimensions(xres, yres);
	if (fb_data->rev > 20)
		epdc_set_update_stride(0);
		epdc_submit_update(fb_data, 0, fb_data->wv_modes.mode_init, UPDATE_MODE_FULL,
		false, true, 0xFF);

	dev_dbg(fb_data->dev, "Mode0 update - Waiting for LUT to complete...\n");

	/* Will timeout after ~4-5 seconds */

	for (i = 0; i < 40; i++) {
		if (!epdc_is_lut_active(0)) {
			dev_dbg(fb_data->dev, "Mode0 init complete\n");
			return;
		}
		msleep(100);
	}

	dev_err(fb_data->dev, "Mode0 init failed!\n");

	return;
}

static void mxc_epdc_fb_fw_handler(const struct firmware *fw,
						     void *context)
{
	struct mxc_epdc_fb_data *fb_data = context;
	int ret;
	struct mxcfb_waveform_data_file *wv_file;
	u8 mode_version;
	int wv_data_offs;
	size_t waveform_data_size;
	int i;
	struct mxcfb_update_data update;
	struct mxcfb_update_marker_data upd_marker_data;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 xres, yres;
	struct clk *epdc_parent;
	unsigned long rounded_parent_rate, epdc_pix_rate,
			rounded_pix_clk, target_pix_clk;

	if (fw == NULL) {
		/* If default FW file load failed, we give up */
		if (fb_data->fw_default_load)
			return;

		/* Try to load default waveform */
		dev_dbg(fb_data->dev,
			"Can't find firmware. Trying fallback fw\n");
		fb_data->fw_default_load = true;
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			"imx/epdc/epdc.fw", fb_data->dev, GFP_KERNEL, fb_data,
			mxc_epdc_fb_fw_handler);
		if (ret)
			dev_err(fb_data->dev,
				"Failed request_firmware_nowait err %d\n", ret);

		return;
	}

	wv_file = (struct mxcfb_waveform_data_file *)fw->data;
	if (fw->size <= sizeof(wv_file->wdh)) {
		dev_err(fb_data->dev, "waveform header is truncated\n");
		release_firmware(fw);
		return;
	}

	mode_version = fw->data[0x10];
	if ((wv_file->wdh.luts & 0x0c) == 0x04)
		fb_data->buf_pix_fmt = EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N;
	else
		fb_data->buf_pix_fmt = EPDC_FORMAT_BUF_PIXEL_FORMAT_P4N;

	/* Netronix mode map used by the E70Q10 0x19 waveform. */
	if (mode_version == 0x20 || mode_version == 0x19 ||
	    mode_version == 0x18 || mode_version == 0x43) {
		fb_data->wv_modes.mode_init = 0;
		fb_data->wv_modes.mode_du = 1;
		fb_data->wv_modes.mode_gc16 = 2;
		fb_data->wv_modes.mode_gc4 = mode_version == 0x18 ? 2 : 7;
		fb_data->wv_modes.mode_gc8 = 2;
		fb_data->wv_modes.mode_gc32 = 2;
		fb_data->wv_modes.mode_gl16 = 3;
		fb_data->wv_modes.mode_aa = 4;
		fb_data->wv_modes.mode_aad = 5;
		fb_data->wv_modes.mode_a2 = 6;
	}
	dev_info(fb_data->dev,
		 "Netronix waveform mode 0x%02x, luts 0x%02x, format %s\n",
		 mode_version, wv_file->wdh.luts,
		 fb_data->buf_pix_fmt == EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N ?
		 "P5N" : "P4N");

	/* Get size and allocate temperature range table */
	fb_data->trt_entries = wv_file->wdh.trc + 1;
	wv_data_offs = sizeof(wv_file->wdh) + fb_data->trt_entries + 1;
	if (fw->size <= wv_data_offs) {
		dev_err(fb_data->dev, "waveform TRT/data is truncated\n");
		release_firmware(fw);
		return;
	}
	fb_data->temp_range_bounds = kzalloc(fb_data->trt_entries + 1,
						GFP_KERNEL);
	if (!fb_data->temp_range_bounds) {
		release_firmware(fw);
		return;
	}

	for (i = 0; i < fb_data->trt_entries; i++)
		dev_dbg(fb_data->dev, "trt entry #%d = 0x%x\n", i, *((u8 *)&wv_file->data + i));

	/* Copy TRT data */
	memcpy(fb_data->temp_range_bounds, &wv_file->data,
		fb_data->trt_entries + 1);

	/* Set default temperature index using TRT and room temp */
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP);

	/* Split Netronix VCD/ACD/XWI tails from the EPDC waveform DMA data. */
	waveform_data_size = fw->size - wv_data_offs;
	ret = netronix_parse_advanced_waveform(fb_data, &wv_file->wdh,
		fw->data + wv_data_offs, waveform_data_size,
		&waveform_data_size);
	if (ret) {
		dev_err(fb_data->dev,
			"invalid Netronix advanced waveform data: %d\n", ret);
		release_firmware(fw);
		return;
	}
	fb_data->waveform_buffer_size = waveform_data_size;

	/* Allocate memory for waveform data */
	fb_data->waveform_buffer_virt = dma_alloc_coherent(fb_data->dev,
						fb_data->waveform_buffer_size,
						&fb_data->waveform_buffer_phys,
						GFP_DMA | GFP_KERNEL);
	if (fb_data->waveform_buffer_virt == NULL) {
		dev_err(fb_data->dev, "Can't allocate mem for waveform!\n");
		return;
	}

	memcpy(fb_data->waveform_buffer_virt, (u8 *)(fw->data) + wv_data_offs,
		fb_data->waveform_buffer_size);

	release_firmware(fw);

	/* Enable clocks to access EPDC regs */
	clk_prepare_enable(fb_data->epdc_clk_axi);

	target_pix_clk = fb_data->cur_mode->vmode->pixclock;
	/* Enable pix clk for EPDC */
	rounded_pix_clk = clk_round_rate(fb_data->epdc_clk_pix, target_pix_clk);

	if (((rounded_pix_clk >= target_pix_clk + target_pix_clk/100) ||
		(rounded_pix_clk <= target_pix_clk - target_pix_clk/100))) {
		/* Can't get close enough without changing parent clk */
		epdc_parent = clk_get_parent(fb_data->epdc_clk_pix);
		rounded_parent_rate = clk_round_rate(epdc_parent, target_pix_clk);

		epdc_pix_rate = target_pix_clk;
		while (epdc_pix_rate < rounded_parent_rate)
			epdc_pix_rate *= 2;
		clk_set_rate(epdc_parent, epdc_pix_rate);

		rounded_pix_clk = clk_round_rate(fb_data->epdc_clk_pix, target_pix_clk);
		if (((rounded_pix_clk >= target_pix_clk + target_pix_clk/100) ||
			(rounded_pix_clk <= target_pix_clk - target_pix_clk/100)))
			/* Still can't get a good clock, provide warning */
			dev_err(fb_data->dev, "Unable to get an accurate EPDC pix clk"
				"desired = %lu, actual = %lu\n", target_pix_clk,
				rounded_pix_clk);
	}

	clk_set_rate(fb_data->epdc_clk_pix, rounded_pix_clk);
	clk_prepare_enable(fb_data->epdc_clk_pix);

	epdc_init_sequence(fb_data);

	/* Disable clocks */
	clk_disable_unprepare(fb_data->epdc_clk_axi);
	clk_disable_unprepare(fb_data->epdc_clk_pix);

	fb_data->hw_ready = true;
	fb_data->hw_initializing = false;

	/*
	 * send_update() validates framebuffer-space coordinates and rotates the
	 * rectangle later in adjust_coordinates().  Native panel dimensions here
	 * make a rotated 1872-wide request fail against the logical 1404-wide fb.
	 */
	xres = screeninfo->xres;
	yres = screeninfo->yres;

	update.update_region.left = 0;
	update.update_region.width = xres;
	update.update_region.top = 0;
	update.update_region.height = yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_marker = INIT_UPDATE_MARKER;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

	upd_marker_data.update_marker = update.update_marker;

	ret = mxc_epdc_fb_send_update(&update, &fb_data->info);
	if (ret) {
		dev_err(fb_data->dev,
			"Initial update submit failed. Error = 0x%x\n", ret);
		return;
	}

	/* Block on initial update */
	ret = mxc_epdc_fb_wait_update_complete(&upd_marker_data,
		&fb_data->info);
	if (ret < 0)
		dev_err(fb_data->dev,
			"Wait for initial update complete failed."
			" Error = 0x%x", ret);
}

static int mxc_epdc_fb_init_hw(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int ret;

	/*
	 * Create fw search string based on ID string in selected videomode.
	 * Format is "imx/epdc_[panel string].fw"
	 */
	if (!fb_data->cur_mode || !fb_data->cur_mode->vmode ||
	    !fb_data->cur_mode->vmode->name)
		return -EINVAL;

	snprintf(fb_data->fw_str, sizeof(fb_data->fw_str),
		 "imx/epdc/epdc_%s.fw",
		 fb_data->cur_mode->vmode->name);

	fb_data->fw_default_load = false;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				fb_data->fw_str, fb_data->dev, GFP_KERNEL,
				fb_data, mxc_epdc_fb_fw_handler);
	if (ret)
		dev_dbg(fb_data->dev,
			"Failed request_firmware_nowait err %d\n", ret);

	return ret;
}

static ssize_t store_update(struct device *device,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct mxcfb_update_data update;
	struct fb_info *info = dev_get_drvdata(device);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int ret;

	memset(&update, 0, sizeof(update));

	if (sysfs_streq(buf, "direct"))
		update.waveform_mode = fb_data->wv_modes.mode_du;
	else if (sysfs_streq(buf, "gc16"))
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
	else if (sysfs_streq(buf, "gc4"))
		update.waveform_mode = fb_data->wv_modes.mode_gc4;
	else
		return -EINVAL;

	/* Now, request full screen update */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = 0;
	update.update_region.height = fb_data->epdc_fb_var.yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.temp = TEMP_USE_AMBIENT;
	update.update_marker = 0;
	update.flags = 0;

	ret = mxc_epdc_fb_send_update(&update, info);
	if (ret)
		return ret;

	return count;
}

static ssize_t page_lifetime_stats_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	unsigned long flags;
	int len, i;
	u64 active_lut_mask = 0;
	u32 pending_count = 0, collision_count = 0, queued_count = 0;
	u32 free_count = 0;
	u32 cur_update_order = 0, cur_newer_order = 0;
	struct update_desc_list *desc;
	struct update_data_list *update;

	(void)attr;
	if (!fb_data->page_lifetime_diag)
		return scnprintf(buf, PAGE_SIZE, "disabled\n");

	spin_lock_irqsave(&fb_data->fb_page_lock, flags);
	len = scnprintf(buf, PAGE_SIZE, "pages=%u", fb_data->fb_page_generation_count);
	for (i = 0; i < fb_data->fb_page_generation_count && len < PAGE_SIZE; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, " p%d_gen=%llu", i,
			(unsigned long long)fb_data->fb_page_generations[i]);
	len += scnprintf(buf + len, PAGE_SIZE - len,
		" source_reused_before_pxp=%llu source_reused_before_copybuf=%llu"
		" untracked_pan=%llu untracked_submit=%llu",
		(unsigned long long)fb_data->source_reused_before_pxp,
		(unsigned long long)fb_data->source_reused_before_copybuf,
		(unsigned long long)fb_data->untracked_fb_pan,
		(unsigned long long)fb_data->untracked_fb_submit);
	spin_unlock_irqrestore(&fb_data->fb_page_lock, flags);

	mutex_lock(&fb_data->queue_mutex);
	list_for_each_entry(desc, &fb_data->upd_pending_list, list)
		pending_count++;
	list_for_each_entry(update, &fb_data->upd_buf_collision_list, list)
		collision_count++;
	list_for_each_entry(update, &fb_data->upd_buf_queue, list)
		queued_count++;
	list_for_each_entry(update, &fb_data->upd_buf_free_list, list)
		free_count++;
	if (fb_data->power_state == POWER_STATE_ON)
		active_lut_mask = epdc_any_luts_active(fb_data->rev);
	if (fb_data->cur_update && fb_data->cur_update->update_desc) {
		cur_update_order = fb_data->cur_update->update_desc->update_order;
		cur_newer_order = fb_data->cur_update->update_desc->
			collided_newer_order;
	}
	len += scnprintf(buf + len, PAGE_SIZE - len,
		" snapshot_enabled=%u snapshot_slots=%u snapshot_copies=%llu"
		" snapshot_hits=%llu snapshot_busy=%llu"
		" collision_created=%llu collision_retry_submitted=%llu"
		" collision_retry_dropped_newer=%llu collision_retry_zero_mask=%llu"
		" collision_newer_completed_before_wb=%llu"
		" collision_retry_max_depth=%llu collision_list_peak=%llu"
		" snapshot_busy_final_candidate=%llu"
		" deferred_latest_retry_armed=%llu"
		" deferred_latest_retry_promoted=%llu"
		" deferred_latest_retry_submitted=%llu"
		" deferred_latest_retry_success=%llu"
		" deferred_latest_retry_replaced=%llu"
		" deferred_latest_retry_dropped_error=%llu"
		" pending_count=%u update_buffers_in_use=%u collision_count=%u"
		" queued_count=%u cur_update=%u cur_update_order=%u"
		" cur_update_newer_order=%u active_lut_mask=0x%llx"
		" deferred_latest_enabled=%u deferred_latest_pending=%u"
		" latest_accepted_generation=%llu latest_submitted_generation=%llu"
		" latest_terminal_desc_generation=%llu"
		" coalesce_passes=%llu coalesce_superseded_descs=%llu"
		" coalesce_superseded_generations=%llu"
		" coalesce_damage_expands=%llu coalesce_full_bounds=%llu"
		" coalesce_skipped_ineligible=%llu coalesce_marker_skips=%llu",
		fb_data->source_snapshot_enabled ? 1 : 0,
		fb_data->source_snapshot_count,
		(unsigned long long)fb_data->source_snapshot_copies,
		(unsigned long long)fb_data->source_snapshot_hits,
		(unsigned long long)fb_data->source_snapshot_busy,
		(unsigned long long)fb_data->collision_created,
		(unsigned long long)fb_data->collision_retry_submitted,
		(unsigned long long)fb_data->collision_retry_dropped_newer,
		(unsigned long long)fb_data->collision_retry_zero_mask,
		(unsigned long long)fb_data->collision_newer_completed_before_wb,
		(unsigned long long)fb_data->collision_retry_max_depth,
		(unsigned long long)fb_data->collision_list_peak,
		(unsigned long long)fb_data->snapshot_busy_final_candidate,
		(unsigned long long)fb_data->deferred_latest_retry_armed,
		(unsigned long long)fb_data->deferred_latest_retry_promoted,
		(unsigned long long)fb_data->deferred_latest_retry_submitted,
		(unsigned long long)fb_data->deferred_latest_retry_success,
		(unsigned long long)fb_data->deferred_latest_retry_replaced,
		(unsigned long long)fb_data->deferred_latest_retry_dropped_error,
		pending_count,
		fb_data->max_num_updates - free_count,
		collision_count,
		queued_count,
		fb_data->cur_update ? 1 : 0,
		cur_update_order,
		cur_newer_order,
		(unsigned long long)active_lut_mask,
		fb_data->deferred_latest_enabled ? 1 : 0,
		fb_data->deferred_latest_desc ? 1 : 0,
		(unsigned long long)fb_data->latest_accepted_generation,
		(unsigned long long)fb_data->latest_submitted_generation,
		(unsigned long long)fb_data->latest_completed_generation,
		(unsigned long long)fb_data->coalesce_passes,
		(unsigned long long)fb_data->coalesce_superseded_descs,
		(unsigned long long)fb_data->coalesce_superseded_generations,
		(unsigned long long)fb_data->coalesce_damage_expands,
		(unsigned long long)fb_data->coalesce_full_bounds,
		(unsigned long long)fb_data->coalesce_skipped_ineligible,
		(unsigned long long)fb_data->coalesce_marker_skips);
	for (i = 0; i < fb_data->source_snapshot_count && len < PAGE_SIZE; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len,
			" s%d_refs=%u", i, fb_data->source_snapshots[i].refs);
	len += scnprintf(buf + len, PAGE_SIZE - len,
		" deferred_latest_refs=%u",
		fb_data->deferred_latest_snapshot.refs);
	mutex_unlock(&fb_data->queue_mutex);

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

static struct device_attribute fb_attrs[] = {
	__ATTR(update, S_IWUSR, NULL, store_update),
};

static struct device_attribute fb_page_lifetime_stats_attr =
	__ATTR(e70q10_page_lifetime_stats, S_IRUGO,
	       page_lifetime_stats_show, NULL);

static const struct of_device_id imx_epdc_dt_ids[] = {
	{ .compatible = "fsl,imx6dl-epdc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_epdc_dt_ids);

/* Restore the E70Q10 values and write order used by the shipping i.MX6SL. */
static int epdc_apply_stock_qos(struct device *dev)
{
	struct device_node *qosc_node;
	struct device_node *ccm_node;
	void __iomem *qosc_base;
	void __iomem *ccm_base;
	u32 ccgr4;

	qosc_node = of_parse_phandle(dev->of_node, "fsl,qosc", 0);
	if (!qosc_node)
		return -ENODEV;
	qosc_base = of_iomap(qosc_node, 0);
	of_node_put(qosc_node);
	if (!qosc_base)
		return -ENOMEM;

	ccm_node = of_find_compatible_node(NULL, NULL, "fsl,imx6sl-ccm");
	if (!ccm_node) {
		iounmap(qosc_base);
		return -ENODEV;
	}
	ccm_base = of_iomap(ccm_node, 0);
	of_node_put(ccm_node);
	if (!ccm_base) {
		iounmap(qosc_base);
		return -ENOMEM;
	}

	/* CCM_CCGR4 at 0x78 gates QOSC while its controls are programmed. */
	ccgr4 = __raw_readl(ccm_base + EPDC_QOSC_CCM_CCGR4_OFFSET);
	__raw_writel(ccgr4 | EPDC_QOSC_CCM_CCGR4_GATE_MASK,
		     ccm_base + EPDC_QOSC_CCM_CCGR4_OFFSET);
	__raw_writel(0, qosc_base);
	__raw_writel(0, qosc_base + EPDC_QOSC_MASTER_ENABLE_OFFSET);
	__raw_writel(0, qosc_base + EPDC_QOSC_EPDC_CHANNEL_OFFSET);
	__raw_writel(0x0f020222, qosc_base +
		     EPDC_QOSC_EPDC_CHANNEL_OFFSET + EPDC_QOSC_WR_OFFSET);
	__raw_writel(0x0f020f22, qosc_base +
		     EPDC_QOSC_EPDC_CHANNEL_OFFSET + EPDC_QOSC_RD_OFFSET);
	/* Preserve the stock readback of the EPDC QoS status register. */
	(void)__raw_readl(qosc_base + EPDC_QOSC_EPDC_CHANNEL_OFFSET +
			  EPDC_QOSC_STATUS_OFFSET);
	__raw_writel(ccgr4, ccm_base + EPDC_QOSC_CCM_CCGR4_OFFSET);

	iounmap(ccm_base);
	iounmap(qosc_base);
	dev_info(dev, "restored stock EPDC QOSC WR/RD priorities\n");
	return 0;
}

int mxc_epdc_fb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pinctrl *pinctrl;
	struct mxc_epdc_fb_data *fb_data;
	struct resource *res;
	struct fb_info *info;
	char *options, *opt;
	char *panel_str = NULL;
	int stats_ret;
	char name[] = "mxcepdcfb";
	struct fb_videomode *vmode;
	int xres_virt, yres_virt, buf_size;
	int xres_virt_rot, yres_virt_rot, pix_size_rot;
	struct fb_var_screeninfo *var_info;
	struct fb_fix_screeninfo *fix_info;
	struct pxp_config_data *pxp_conf;
	struct pxp_proc_data *proc_data;
	struct scatterlist *sg;
	struct update_data_list *upd_list;
	struct update_data_list *plist, *temp_list;
	int i;
	unsigned long x_mem_size = 0;
	u32 val;
	int irq;

	fb_data = (struct mxc_epdc_fb_data *)framebuffer_alloc(
			sizeof(struct mxc_epdc_fb_data), &pdev->dev);
	if (fb_data == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	/* Get platform data and check validity */
	fb_data->pdata = &epdc_data;
	if ((fb_data->pdata == NULL) || (fb_data->pdata->num_modes < 1)
		|| (fb_data->pdata->epdc_mode == NULL)
		|| (fb_data->pdata->epdc_mode->vmode == NULL)) {
		ret = -EINVAL;
		goto out_fbdata;
	}

	if (fb_get_options(name, &options)) {
		ret = -ENODEV;
		goto out_fbdata;
	}

	fb_data->tce_prevent = 0;

	if (options)
		while ((opt = strsep(&options, ",")) != NULL) {
			if (!*opt)
				continue;

			if (!strncmp(opt, "bpp=", 4))
				fb_data->default_bpp =
					simple_strtoul(opt + 4, NULL, 0);
			else if (!strncmp(opt, "x_mem=", 6))
				x_mem_size = memparse(opt + 6, NULL);
			else if (!strncmp(opt, "tce_prevent", 11))
				fb_data->tce_prevent = 1;
			else
				panel_str = opt;
		}

	fb_data->dev = &pdev->dev;
	fb_data->page_lifetime_diag =
		of_machine_is_compatible("ridi,e70q10");
	spin_lock_init(&fb_data->fb_page_lock);

	if (!fb_data->default_bpp)
		fb_data->default_bpp = 16;

	/* Stock U-Boot hard-codes E060SCM in ATAGs.  The E70Q10 panel is
	 * ED078KH1, so use this board's first (ED078KH1) mode regardless of
	 * that legacy command-line panel name.
	 */
	if (of_machine_is_compatible("ridi,e70q10"))
		panel_str = NULL;

	/* Set default (first defined mode) before searching for a match */
	fb_data->cur_mode = &fb_data->pdata->epdc_mode[0];

	if (panel_str)
		for (i = 0; i < fb_data->pdata->num_modes; i++)
			if (!strcmp(fb_data->pdata->epdc_mode[i].vmode->name,
						panel_str)) {
				fb_data->cur_mode =
					&fb_data->pdata->epdc_mode[i];
				break;
			}

	vmode = fb_data->cur_mode->vmode;

	platform_set_drvdata(pdev, fb_data);
	info = &fb_data->info;

	/* Allocate color map for the FB */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto out_fbdata;

	dev_dbg(&pdev->dev, "resolution %dx%d, bpp %d\n",
		vmode->xres, vmode->yres, fb_data->default_bpp);

	/*
	 * GPU alignment restrictions dictate framebuffer parameters:
	 * - 32-byte alignment for buffer width
	 * - 128-byte alignment for buffer height
	 * => 4K buffer alignment for buffer start
	 */
	xres_virt = ALIGN(vmode->xres, 32);
	yres_virt = ALIGN(vmode->yres, 128);
	fb_data->max_pix_size = PAGE_ALIGN(xres_virt * yres_virt);

	/*
	 * Have to check to see if aligned buffer size when rotated
	 * is bigger than when not rotated, and use the max
	 */
	xres_virt_rot = ALIGN(vmode->yres, 32);
	yres_virt_rot = ALIGN(vmode->xres, 128);
	pix_size_rot = PAGE_ALIGN(xres_virt_rot * yres_virt_rot);
	fb_data->max_pix_size = (fb_data->max_pix_size > pix_size_rot) ?
				fb_data->max_pix_size : pix_size_rot;

	buf_size = fb_data->max_pix_size * fb_data->default_bpp/8;

	/* Compute the number of screens needed based on X memory requested */
	if (x_mem_size > 0) {
		fb_data->num_screens = DIV_ROUND_UP(x_mem_size, buf_size);
		if (fb_data->num_screens < NUM_SCREENS_MIN)
			fb_data->num_screens = NUM_SCREENS_MIN;
		else if (buf_size * fb_data->num_screens > SZ_16M)
			fb_data->num_screens = SZ_16M / buf_size;
	} else
		fb_data->num_screens = NUM_SCREENS_MIN;

	fb_data->map_size = buf_size * fb_data->num_screens;
	if (fb_data->page_lifetime_diag) {
		fb_data->fb_page_generations = devm_kcalloc(&pdev->dev,
				fb_data->num_screens,
				sizeof(*fb_data->fb_page_generations), GFP_KERNEL);
		if (!fb_data->fb_page_generations) {
			dev_warn(&pdev->dev,
				 "page lifetime diagnostic disabled: generation allocation failed\n");
			fb_data->page_lifetime_diag = false;
		} else {
			fb_data->fb_page_generation_count = fb_data->num_screens;
		}
	}
	dev_dbg(&pdev->dev, "memory to allocate: %d\n", fb_data->map_size);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto out_cmap;
	}

	epdc_base = devm_ioremap_resource(&pdev->dev, res);
	if (epdc_base == NULL) {
		ret = -ENOMEM;
		goto out_cmap;
	}

	/* Allocate FB memory */
	info->screen_base = dma_alloc_writecombine(&pdev->dev,
						  fb_data->map_size,
						  &fb_data->phys_start,
						  GFP_DMA | GFP_KERNEL);

	if (info->screen_base == NULL) {
		ret = -ENOMEM;
		goto out_cmap;
	}

	/*
	 * An EPDC keeps displaying its previous contents until an explicit
	 * update completes.  dma_alloc_writecombine() does not promise a useful
	 * initial image, and an all-zero RGB565 buffer is black.  That made the
	 * E70Q10 flash (and sometimes remain) black between the boot stages.
	 * Start every framebuffer page as white so an early or missed update is
	 * harmless on the E-Ink panel.
	 */
	if (of_machine_is_compatible("ridi,e70q10"))
		memset(info->screen_base, 0xff, fb_data->map_size);

	dev_dbg(&pdev->dev, "allocated at %p:0x%x\n", info->screen_base,
		fb_data->phys_start);

	var_info = &info->var;
	var_info->activate = FB_ACTIVATE_TEST;
	var_info->bits_per_pixel = fb_data->default_bpp;
	var_info->xres = vmode->xres;
	var_info->yres = vmode->yres;
	var_info->xres_virtual = xres_virt;
	/* Additional screens allow for panning  and buffer flipping */
	var_info->yres_virtual = yres_virt * fb_data->num_screens;

	/*
	 * The ED078KH1 timing is wired to the EPDC as 1872x1404, but the
	 * E70Q10 is a portrait reader.  This driver already supports a rotated
	 * framebuffer: update regions are converted by adjust_coordinates() and
	 * PxP rotates the source before it reaches the native EPDC working
	 * buffer.  Expose that supported 1404x1872 mode from probe time so Android
	 * never discovers a landscape natural display and never has to apply a
	 * framework/HWC rotation of its own.
	 */
	if (of_machine_is_compatible("ridi,e70q10")) {
		var_info->rotate = FB_ROTATE_CCW;
		var_info->xres = vmode->yres;
		var_info->yres = vmode->xres;
		var_info->xres_virtual = xres_virt_rot;
		var_info->yres_virtual = yres_virt_rot * fb_data->num_screens;
		dev_info(&pdev->dev,
			 "E70Q10 portrait fb %ux%u virtual %ux%u rotate=%u; native EPDC %ux%u\n",
			 var_info->xres, var_info->yres,
			 var_info->xres_virtual, var_info->yres_virtual,
			 var_info->rotate, vmode->xres, vmode->yres);
	}

	var_info->pixclock = vmode->pixclock;
	var_info->left_margin = vmode->left_margin;
	var_info->right_margin = vmode->right_margin;
	var_info->upper_margin = vmode->upper_margin;
	var_info->lower_margin = vmode->lower_margin;
	var_info->hsync_len = vmode->hsync_len;
	var_info->vsync_len = vmode->vsync_len;
	var_info->vmode = FB_VMODE_NONINTERLACED;

	switch (fb_data->default_bpp) {
	case 32:
	case 24:
		var_info->red.offset = 16;
		var_info->red.length = 8;
		var_info->green.offset = 8;
		var_info->green.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.length = 8;
		break;

	case 16:
		var_info->red.offset = 11;
		var_info->red.length = 5;
		var_info->green.offset = 5;
		var_info->green.length = 6;
		var_info->blue.offset = 0;
		var_info->blue.length = 5;
		break;

	case 8:
		/*
		 * For 8-bit grayscale, R, G, and B offset are equal.
		 *
		 */
		var_info->grayscale = GRAYSCALE_8BIT;

		var_info->red.length = 8;
		var_info->red.offset = 0;
		var_info->red.msb_right = 0;
		var_info->green.length = 8;
		var_info->green.offset = 0;
		var_info->green.msb_right = 0;
		var_info->blue.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.msb_right = 0;
		break;

	default:
		dev_err(&pdev->dev, "unsupported bitwidth %d\n",
			fb_data->default_bpp);
		ret = -EINVAL;
		goto out_dma_fb;
	}

	fix_info = &info->fix;

	strcpy(fix_info->id, "mxc_epdc_fb");
	fix_info->type = FB_TYPE_PACKED_PIXELS;
	fix_info->visual = FB_VISUAL_TRUECOLOR;
	fix_info->xpanstep = 0;
	fix_info->ypanstep = 0;
	fix_info->ywrapstep = 0;
	fix_info->accel = FB_ACCEL_NONE;
	fix_info->smem_start = fb_data->phys_start;
	fix_info->smem_len = fb_data->map_size;
	fix_info->ypanstep = 0;

	fb_data->native_width = vmode->xres;
	fb_data->native_height = vmode->yres;

	info->fbops = &mxc_epdc_fb_ops;
	info->var.activate = FB_ACTIVATE_NOW;
	info->pseudo_palette = fb_data->pseudo_palette;
	info->screen_size = info->fix.smem_len;
	info->flags = FBINFO_FLAG_DEFAULT;

	mxc_epdc_fb_set_fix(info);

	fb_data->auto_mode = AUTO_UPDATE_MODE_REGION_MODE;
	fb_data->upd_scheme = UPDATE_SCHEME_QUEUE_AND_MERGE;

	/* Initialize our internal copy of the screeninfo */
	fb_data->epdc_fb_var = *var_info;
	fb_data->fb_offset = 0;
	fb_data->eof_sync_period = 0;

	fb_data->epdc_clk_axi = clk_get(fb_data->dev, "epdc_axi");
	if (IS_ERR(fb_data->epdc_clk_axi)) {
		dev_err(&pdev->dev, "Unable to get EPDC AXI clk."
			"err = %d\n", (int)fb_data->epdc_clk_axi);
		ret = -ENODEV;
		goto out_dma_fb;
	}
	fb_data->epdc_clk_pix = clk_get(fb_data->dev, "epdc_pix");
	if (IS_ERR(fb_data->epdc_clk_pix)) {
		dev_err(&pdev->dev, "Unable to get EPDC pix clk."
			"err = %d\n", (int)fb_data->epdc_clk_pix);
		ret = -ENODEV;
		goto out_dma_fb;
	}

	clk_prepare_enable(fb_data->epdc_clk_axi);
	val = __raw_readl(EPDC_VERSION);
	if (of_machine_is_compatible("ridi,e70q10")) {
		int qos_ret = epdc_apply_stock_qos(&pdev->dev);

		if (qos_ret)
			dev_warn(&pdev->dev,
				 "stock EPDC QoS setup unavailable: %d\n", qos_ret);
	}
	clk_disable_unprepare(fb_data->epdc_clk_axi);
	fb_data->rev = ((val & EPDC_VERSION_MAJOR_MASK) >>
				EPDC_VERSION_MAJOR_OFFSET) * 10
			+ ((val & EPDC_VERSION_MINOR_MASK) >>
				EPDC_VERSION_MINOR_OFFSET);
	dev_dbg(&pdev->dev, "EPDC version = %d\n", fb_data->rev);

	if (fb_data->rev < 20) {
		fb_data->num_luts = EPDC_V1_NUM_LUTS;
		fb_data->max_num_updates = EPDC_V1_MAX_NUM_UPDATES;
	} else {
		fb_data->num_luts = EPDC_V2_NUM_LUTS;
		fb_data->max_num_updates = EPDC_V2_MAX_NUM_UPDATES;
		if (vmode->xres > EPDC_V2_MAX_UPDATE_WIDTH)
			fb_data->restrict_width = true;
	}
	fb_data->max_num_buffers = EPDC_MAX_NUM_BUFFERS;

	/*
	 * Initialize lists for pending updates,
	 * active update requests, update collisions,
	 * and freely available updates.
	 */
	INIT_LIST_HEAD(&fb_data->upd_pending_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_queue);
	INIT_LIST_HEAD(&fb_data->upd_buf_free_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_collision_list);

	/* Allocate update buffers and add them to the list */
	for (i = 0; i < fb_data->max_num_updates; i++) {
		upd_list = kzalloc(sizeof(*upd_list), GFP_KERNEL);
		if (upd_list == NULL) {
			ret = -ENOMEM;
			goto out_upd_lists;
		}

		/* Add newly allocated buffer to free list */
		list_add(&upd_list->list, &fb_data->upd_buf_free_list);
	}

	/*
	 * Android HWC1 releases framebuffer targets without an EPDC source fence.
	 * Keep a small bounded pool of immutable full-frame sources for queued
	 * E70Q10 updates. Three normal slots match the Android framebuffer page
	 * count. A separate one-deep reserve retains the newest eligible ordinary
	 * update when all normal slots are owned by live descriptors.
	 */
	if (of_machine_is_compatible("ridi,e70q10")) {
		fb_data->source_snapshot_size =
			fb_data->max_pix_size * fb_data->default_bpp / 8;
		fb_data->source_snapshot_copy_size =
			(size_t)fb_data->info.fix.line_length *
			fb_data->info.var.yres;
		if (!fb_data->source_snapshot_copy_size ||
		    fb_data->source_snapshot_copy_size >
			fb_data->source_snapshot_size) {
			dev_err(&pdev->dev,
				"invalid E70Q10 snapshot geometry copy=%zu alloc=%zu\n",
				fb_data->source_snapshot_copy_size,
				fb_data->source_snapshot_size);
			ret = -EINVAL;
			goto out_upd_lists;
		}

		fb_data->source_snapshots = kcalloc(
			E70Q10_SOURCE_SNAPSHOT_SLOTS,
			sizeof(*fb_data->source_snapshots), GFP_KERNEL);
		if (!fb_data->source_snapshots) {
			ret = -ENOMEM;
			goto out_upd_lists;
		}

		for (i = 0; i < E70Q10_SOURCE_SNAPSHOT_SLOTS; i++) {
			struct e70q10_source_snapshot *snapshot =
				&fb_data->source_snapshots[i];

			snapshot->virt_addr = dma_alloc_coherent(fb_data->dev,
				fb_data->source_snapshot_size,
				&snapshot->phys_addr, GFP_DMA | GFP_KERNEL);
			if (!snapshot->virt_addr) {
				dev_warn(&pdev->dev,
					"E70Q10 source snapshot pool reduced to %u slots\n",
					fb_data->source_snapshot_count);
				break;
			}
			memset(snapshot->virt_addr, 0xff,
			       fb_data->source_snapshot_size);
			fb_data->source_snapshot_count++;
		}

		if (!fb_data->source_snapshot_count) {
			dev_err(&pdev->dev,
				"unable to allocate any E70Q10 source snapshot slot\n");
			ret = -ENOMEM;
			goto out_source_snapshots;
		}
		fb_data->source_snapshot_enabled = true;
		fb_data->deferred_latest_snapshot.virt_addr =
			dma_alloc_coherent(fb_data->dev,
				fb_data->source_snapshot_size,
				&fb_data->deferred_latest_snapshot.phys_addr,
				GFP_DMA | GFP_KERNEL);
		if (!fb_data->deferred_latest_snapshot.virt_addr) {
			dev_warn(&pdev->dev,
				 "E70Q10 deferred latest-state reserve unavailable; "
				 "snapshot pressure can still return -EBUSY\n");
		} else {
			memset(fb_data->deferred_latest_snapshot.virt_addr,
			       0xff, fb_data->source_snapshot_size);
			fb_data->deferred_latest_enabled = true;
		}
		dev_info(&pdev->dev,
			 "E70Q10 immutable source snapshots: slots=%u copy=%zu alloc=%zu deferred=%u\n",
			 fb_data->source_snapshot_count,
			 fb_data->source_snapshot_copy_size,
			 fb_data->source_snapshot_size,
			 fb_data->deferred_latest_enabled ? 1 : 0);
	}

	fb_data->virt_addr_updbuf =
		kzalloc(sizeof(void *) * fb_data->max_num_buffers, GFP_KERNEL);
	fb_data->phys_addr_updbuf =
		kzalloc(sizeof(dma_addr_t) * fb_data->max_num_buffers,
			GFP_KERNEL);
	if (!fb_data->virt_addr_updbuf || !fb_data->phys_addr_updbuf) {
		ret = -ENOMEM;
		goto out_upd_buffers;
	}
	for (i = 0; i < fb_data->max_num_buffers; i++) {
		/*
		 * Allocate memory for PxP output buffer.
		 * Each update buffer is 1 byte per pixel, and can
		 * be as big as the full-screen frame buffer
		 */
		/*
		 * PxP writes this buffer and EPDC immediately consumes it.  A
		 * kmalloc()+virt_to_phys() buffer is not a valid coherent DMA
		 * hand-off on i.MX6SL; the Netronix reference compensates with
		 * global inner/outer cache flushes.  Allocate it coherently instead
		 * so neither engine can observe stale cache lines.
		 */
		fb_data->virt_addr_updbuf[i] = dma_alloc_coherent(fb_data->dev,
			fb_data->max_pix_size, &fb_data->phys_addr_updbuf[i],
			GFP_DMA | GFP_KERNEL);
		if (fb_data->virt_addr_updbuf[i] == NULL) {
			ret = -ENOMEM;
			goto out_upd_buffers;
		}
		memset(fb_data->virt_addr_updbuf[i], 0xf0,
		       fb_data->max_pix_size);

	}

	/* Counter indicating which update buffer should be used next. */
	fb_data->upd_buffer_num = 0;

	/*
	 * Allocate memory for PxP SW workaround buffer
	 * These buffers are used to hold copy of the update region,
	 * before sending it to PxP for processing.
	 */
	fb_data->virt_addr_copybuf =
	    dma_alloc_coherent(fb_data->info.device, fb_data->max_pix_size*2,
			       &fb_data->phys_addr_copybuf,
			       GFP_DMA | GFP_KERNEL);
	if (fb_data->virt_addr_copybuf == NULL) {
		ret = -ENOMEM;
		goto out_upd_buffers;
	}

	fb_data->working_buffer_size = vmode->yres * vmode->xres * 2;
	/* Allocate memory for EPDC working buffer */
	fb_data->working_buffer_virt =
	    dma_alloc_coherent(&pdev->dev, fb_data->working_buffer_size,
			       &fb_data->working_buffer_phys,
			       GFP_DMA | GFP_KERNEL);
	if (fb_data->working_buffer_virt == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for working buf!\n");
		ret = -ENOMEM;
		goto out_copybuffer;
	}

	/* Initialize EPDC pins */
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "can't get/select pinctrl\n");
		ret = PTR_ERR(pinctrl);
		goto out_copybuffer;
	}

	fb_data->in_init = false;

	fb_data->hw_ready = false;
	fb_data->hw_initializing = false;

	/*
	 * Set default waveform mode values.
	 * Should be overwritten via ioctl.
	 */
	fb_data->wv_modes.mode_init = 0;
	fb_data->wv_modes.mode_du = 1;
	fb_data->wv_modes.mode_gc4 = 3;
	fb_data->wv_modes.mode_gc8 = 2;
	fb_data->wv_modes.mode_gc16 = 2;
	fb_data->wv_modes.mode_gc32 = 2;
	fb_data->wv_modes.mode_aa = 2;
	fb_data->wv_modes.mode_aad = 2;
	fb_data->wv_modes.mode_gl16 = 2;
	fb_data->wv_modes.mode_a2 = 1;
	fb_data->wv_modes_update = true;
	fb_data->buf_pix_fmt = EPDC_FORMAT_BUF_PIXEL_FORMAT_P4N;

	/* Initialize marker list */
	INIT_LIST_HEAD(&fb_data->full_marker_list);
	INIT_LIST_HEAD(&fb_data->sleep_marker_done_list);
	init_waitqueue_head(&fb_data->sleep_barrier_waitq);
	atomic_set(&fb_data->sleep_barrier_event, 0);

	/* Initialize all LUTs to inactive */
	fb_data->lut_update_order =
		kzalloc(fb_data->num_luts * sizeof(u32 *), GFP_KERNEL);
	for (i = 0; i < fb_data->num_luts; i++)
		fb_data->lut_update_order[i] = 0;

	INIT_DELAYED_WORK(&fb_data->epdc_done_work, epdc_done_work_func);
	fb_data->epdc_submit_workqueue = alloc_workqueue("EPDC Submit",
					WQ_MEM_RECLAIM | WQ_HIGHPRI |
					WQ_CPU_INTENSIVE | WQ_UNBOUND, 1);
	INIT_WORK(&fb_data->epdc_submit_work, epdc_submit_work_func);
	fb_data->epdc_intr_workqueue = alloc_workqueue("EPDC Interrupt",
					WQ_MEM_RECLAIM | WQ_HIGHPRI |
					WQ_CPU_INTENSIVE | WQ_UNBOUND, 1);
	INIT_WORK(&fb_data->epdc_intr_work, epdc_intr_work_func);
	INIT_WORK(&fb_data->epdc_recovery_work, epdc_recovery_work_func);

	/* Retrieve EPDC IRQ num */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	fb_data->epdc_irq = irq;

	/* Register IRQ handler */
	ret = devm_request_irq(&pdev->dev, fb_data->epdc_irq,
				mxc_epdc_irq_handler, 0, "epdc", fb_data);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
			fb_data->epdc_irq, ret);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}

#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
	info->fbdefio = &mxc_epdc_fb_defio;
	fb_deferred_io_init(info);
#else
	/* fbmem checks fbdefio even when automatic updates are disabled.  Do
	 * not expose an uninitialized delayed_work object in manual-update mode.
	 */
	info->fbdefio = NULL;
#endif

	/* get pmic regulators */
	fb_data->display_regulator = devm_regulator_get(&pdev->dev, "DISPLAY");
	if (IS_ERR(fb_data->display_regulator)) {
		dev_err(&pdev->dev, "Unable to get display PMIC regulator."
			"err = 0x%x\n", (int)fb_data->display_regulator);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	fb_data->vcom_regulator = devm_regulator_get(&pdev->dev, "VCOM");
	if (IS_ERR(fb_data->vcom_regulator)) {
		dev_err(&pdev->dev, "Unable to get VCOM regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	fb_data->v3p3_regulator = devm_regulator_get(&pdev->dev, "V3P3");
	if (IS_ERR(fb_data->v3p3_regulator)) {
		dev_err(&pdev->dev, "Unable to get V3P3 regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}

	fb_data->cur_update = NULL;
	fb_data->cleanup_fence_lut = INVALID_LUT;

	mutex_init(&fb_data->queue_mutex);
	mutex_init(&fb_data->pxp_mutex);
	mutex_init(&fb_data->power_mutex);
	mutex_init(&fb_data->pm_control_mutex);
	spin_lock_init(&fb_data->epdc_recovery_lock);
	atomic_set(&fb_data->epdc_recovery_state, 0);
	atomic_set(&fb_data->tce_underrun_count, 0);
	init_completion(&fb_data->epdc_recovery_done);
	complete_all(&fb_data->epdc_recovery_done);

	/*
	 * Fill out PxP config data structure based on FB info and
	 * processing tasks required
	 */
	pxp_conf = &fb_data->pxp_conf;
	proc_data = &pxp_conf->proc_data;

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = fb_data->info.var.xres;
	proc_data->drect.height = proc_data->srect.height = fb_data->info.var.yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = 0;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;
	proc_data->lut_map = NULL;

	/*
	 * We initially configure PxP for RGB->YUV conversion,
	 * and only write out Y component of the result.
	 */

	/*
	 * Initialize S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
	pxp_conf->s0_param.width = fb_data->info.var.xres_virtual;
	pxp_conf->s0_param.height = fb_data->info.var.yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize OL0 channel parameters
	 * No overlay will be used for PxP operation
	 */
	pxp_conf->ol_param[0].combine_enable = false;
	pxp_conf->ol_param[0].width = 0;
	pxp_conf->ol_param[0].height = 0;
	pxp_conf->ol_param[0].pixel_fmt = PXP_PIX_FMT_RGB565;
	pxp_conf->ol_param[0].color_key_enable = false;
	pxp_conf->ol_param[0].color_key = -1;
	pxp_conf->ol_param[0].global_alpha_enable = false;
	pxp_conf->ol_param[0].global_alpha = 0;
	pxp_conf->ol_param[0].local_alpha_enable = false;

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = fb_data->info.var.xres;
	pxp_conf->out_param.height = fb_data->info.var.yres;
	pxp_conf->out_param.stride = pxp_conf->out_param.width;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	/* Initialize color map for conversion of 8-bit gray pixels */
	fb_data->pxp_conf.proc_data.lut_map = kmalloc(256, GFP_KERNEL);
	if (fb_data->pxp_conf.proc_data.lut_map == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for lut map!\n");
		ret = -ENOMEM;
		goto out_dma_work_buf;
	}
	for (i = 0; i < 256; i++)
		fb_data->pxp_conf.proc_data.lut_map[i] = i;

	fb_data->pxp_conf.proc_data.lut_map_updated = true;

	/*
	 * Ensure this is set to NULL here...we will initialize pxp_chan
	 * later in our thread.
	 */
	fb_data->pxp_chan = NULL;

	/* Initialize Scatter-gather list containing 2 buffer addresses. */
	sg = fb_data->sg;
	sg_init_table(sg, 2);

	/*
	 * For use in PxP transfers:
	 * sg[0] holds the FB buffer pointer
	 * sg[1] holds the Output buffer pointer (configured before TX request)
	 */
	sg_dma_address(&sg[0]) = info->fix.smem_start;
	sg_set_page(&sg[0], virt_to_page(info->screen_base),
		    info->fix.smem_len, offset_in_page(info->screen_base));

	fb_data->order_cnt = 0;
	fb_data->waiting_for_wb = false;
	fb_data->waiting_for_lut = false;
	fb_data->waiting_for_lut15 = false;
	fb_data->waiting_for_idle = false;
	fb_data->blank = FB_BLANK_UNBLANK;
	fb_data->last_panel_powerup_result = -ENODATA;
	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;
	fb_data->boot_initial_powerdown_pending = true;
	fb_data->wait_for_powerdown = false;
	fb_data->updates_active = false;
	fb_data->pwrdown_delay = 0;

	/* Register FB */
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev,
			"register_framebuffer failed with error %d\n", ret);
		goto out_lutmap;
	}

	/* register_framebuffer() creates info->dev (/sys/class/graphics/fbN). */
	ret = device_create_file(info->dev, &fb_attrs[0]);
	if (ret) {
		dev_err(&pdev->dev,
			"Unable to create /sys/class/graphics/fb%d/update: %d\n",
			info->node, ret);
		unregister_framebuffer(info);
		goto out_lutmap;
	}
	if (fb_data->page_lifetime_diag) {
		stats_ret = device_create_file(info->dev,
				&fb_page_lifetime_stats_attr);
		if (stats_ret) {
			dev_warn(&pdev->dev,
				 "page lifetime stats unavailable: %d\n", stats_ret);
		} else {
			fb_data->page_lifetime_stats_created = true;
		}
	}


	g_fb_data = fb_data;

	pm_runtime_enable(fb_data->dev);

#ifdef DEFAULT_PANEL_HW_INIT
	ret = mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize HW!\n");
	}
#endif

	goto out;

remove_fb_attrs:
	if (fb_data->page_lifetime_stats_created) {
		device_remove_file(info->dev, &fb_page_lifetime_stats_attr);
		fb_data->page_lifetime_stats_created = false;
	}
	device_remove_file(info->dev, &fb_attrs[0]);
	unregister_framebuffer(info);
	platform_set_drvdata(pdev, NULL);

out_lutmap:
	kfree(fb_data->pxp_conf.proc_data.lut_map);
out_dma_work_buf:
	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
		fb_data->working_buffer_virt, fb_data->working_buffer_phys);
out_copybuffer:
	dma_free_writecombine(&pdev->dev, fb_data->max_pix_size*2,
			      fb_data->virt_addr_copybuf,
			      fb_data->phys_addr_copybuf);
out_upd_buffers:
	if (fb_data->virt_addr_updbuf && fb_data->phys_addr_updbuf)
		for (i = 0; i < fb_data->max_num_buffers; i++)
			if (fb_data->virt_addr_updbuf[i] != NULL)
				dma_free_coherent(fb_data->dev, fb_data->max_pix_size,
					fb_data->virt_addr_updbuf[i],
					fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);
out_source_snapshots:
	if (fb_data->deferred_latest_snapshot.virt_addr) {
		dma_free_coherent(fb_data->dev,
			fb_data->source_snapshot_size,
			fb_data->deferred_latest_snapshot.virt_addr,
			fb_data->deferred_latest_snapshot.phys_addr);
		fb_data->deferred_latest_snapshot.virt_addr = NULL;
	}
	if (fb_data->source_snapshots) {
		for (i = 0; i < fb_data->source_snapshot_count; i++)
			if (fb_data->source_snapshots[i].virt_addr)
				dma_free_coherent(fb_data->dev,
					fb_data->source_snapshot_size,
					fb_data->source_snapshots[i].virt_addr,
					fb_data->source_snapshots[i].phys_addr);
		kfree(fb_data->source_snapshots);
		fb_data->source_snapshots = NULL;
		fb_data->source_snapshot_count = 0;
	}
out_upd_lists:
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}
out_dma_fb:
	dma_free_writecombine(&pdev->dev, fb_data->map_size, info->screen_base,
			      fb_data->phys_start);

out_cmap:
	fb_dealloc_cmap(&info->cmap);
out_fbdata:
	kfree(fb_data);
out:
	return ret;
}

static int mxc_epdc_fb_remove(struct platform_device *pdev)
{
	struct update_data_list *plist, *temp_list;
	struct mxc_epdc_fb_data *fb_data = platform_get_drvdata(pdev);
	int i;

	mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &fb_data->info);
	disable_irq(fb_data->epdc_irq);
	cancel_work_sync(&fb_data->epdc_recovery_work);
	cancel_work_sync(&fb_data->epdc_intr_work);

	flush_workqueue(fb_data->epdc_submit_workqueue);
	destroy_workqueue(fb_data->epdc_submit_workqueue);

	if (fb_data->page_lifetime_stats_created)
		device_remove_file(fb_data->info.dev,
				  &fb_page_lifetime_stats_attr);
	device_remove_file(fb_data->info.dev, &fb_attrs[0]);
	unregister_framebuffer(&fb_data->info);

	for (i = 0; i < fb_data->max_num_buffers; i++)
		if (fb_data->virt_addr_updbuf[i] != NULL)
			dma_free_coherent(fb_data->dev, fb_data->max_pix_size,
				fb_data->virt_addr_updbuf[i],
				fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);

	if (fb_data->source_snapshots) {
		for (i = 0; i < fb_data->source_snapshot_count; i++)
			if (fb_data->source_snapshots[i].virt_addr)
				dma_free_coherent(fb_data->dev,
					fb_data->source_snapshot_size,
					fb_data->source_snapshots[i].virt_addr,
					fb_data->source_snapshots[i].phys_addr);
		kfree(fb_data->source_snapshots);
	}
	if (fb_data->deferred_latest_snapshot.virt_addr)
		dma_free_coherent(fb_data->dev,
			fb_data->source_snapshot_size,
			fb_data->deferred_latest_snapshot.virt_addr,
			fb_data->deferred_latest_snapshot.phys_addr);

	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
				fb_data->working_buffer_virt,
				fb_data->working_buffer_phys);
	if (fb_data->waveform_buffer_virt != NULL)
		dma_free_writecombine(&pdev->dev, fb_data->waveform_buffer_size,
				fb_data->waveform_buffer_virt,
				fb_data->waveform_buffer_phys);
	kfree(fb_data->waveform_vcd_buffer);
	kfree(fb_data->waveform_acd_buffer);
	kfree(fb_data->waveform_xwi_buffer);
	if (fb_data->virt_addr_copybuf != NULL)
		dma_free_writecombine(&pdev->dev, fb_data->max_pix_size*2,
				fb_data->virt_addr_copybuf,
				fb_data->phys_addr_copybuf);
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}
#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
	fb_deferred_io_cleanup(&fb_data->info);
#endif

	dma_free_writecombine(&pdev->dev, fb_data->map_size, fb_data->info.screen_base,
			      fb_data->phys_start);

	/* Release PxP-related resources */
	if (fb_data->pxp_chan != NULL)
		dma_release_channel(&fb_data->pxp_chan->dma_chan);

	fb_dealloc_cmap(&fb_data->info.cmap);

	framebuffer_release(&fb_data->info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxc_epdc_fb_suspend(struct device *dev)
{
	struct mxc_epdc_fb_data *data = dev_get_drvdata(dev);
	int saved_pwrdown_delay;
	int ret;

	mutex_lock(&data->pm_control_mutex);
	mutex_lock(&data->queue_mutex);
	saved_pwrdown_delay = data->pwrdown_delay;
	data->system_suspend_in_progress = true;
	data->suspend_blocked = true;
	data->pwrdown_delay = FB_POWERDOWN_DISABLE;
	mutex_unlock(&data->queue_mutex);

	/* Stop a previously scheduled automatic shutdown before draining. */
	cancel_delayed_work_sync(&data->epdc_done_work);
	ret = mxc_epdc_fb_flush_updates(data);
	if (ret)
		goto abort_suspend;

	/* An underrun recovery is allowed to finish its own repair, but a plain
	 * software flush timeout must never trigger a reset while a LUT is live.
	 */
	if (atomic_read(&data->epdc_recovery_state)) {
		if (!wait_for_completion_timeout(&data->epdc_recovery_done,
					msecs_to_jiffies(20000))) {
			ret = -ETIMEDOUT;
			goto abort_suspend;
		}
		ret = data->epdc_recovery_result;
		if (ret)
			goto abort_suspend;
		ret = mxc_epdc_fb_flush_updates(data);
		if (ret)
			goto abort_suspend;
	}

	ret = epdc_wait_for_hw_idle(data, true, 5000, NULL);
	if (ret)
		goto abort_suspend;

	ret = mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &data->info);
	if (ret)
		goto abort_suspend;

	mutex_lock(&data->queue_mutex);
	data->pwrdown_delay = saved_pwrdown_delay;
	mutex_unlock(&data->queue_mutex);
	mutex_unlock(&data->pm_control_mutex);
	return 0;

abort_suspend:
	mutex_lock(&data->queue_mutex);
	data->pwrdown_delay = saved_pwrdown_delay;
	data->suspend_blocked = false;
	data->system_suspend_in_progress = false;
	mutex_unlock(&data->queue_mutex);
	mutex_lock(&data->power_mutex);
	if (data->power_state == POWER_STATE_ON)
		data->powering_down = false;
	mutex_unlock(&data->power_mutex);
	mutex_unlock(&data->pm_control_mutex);
	return ret;
}

static int mxc_epdc_fb_resume(struct device *dev)
{
	struct mxc_epdc_fb_data *data = dev_get_drvdata(dev);
	int ret;

	ret = mxc_epdc_fb_blank(FB_BLANK_UNBLANK, &data->info);
	if (ret) {
		epdc_recovery_dump(data, "resume-unblank-failed");
		epdc_finish_recovery(data, ret);
		return ret;
	}
	ret = epdc_init_settings_runtime(data);
	if (ret)
		goto resume_init_failed;

	mutex_lock(&data->queue_mutex);
	if (!list_empty(&data->upd_pending_list) ||
	    !list_empty(&data->upd_buf_queue) ||
	    !list_empty(&data->upd_buf_collision_list) || data->cur_update ||
	    data->updates_active) {
		mutex_unlock(&data->queue_mutex);
		dev_err(data->dev,
			"EPDC resume found undrained software update state\n");
		epdc_finish_recovery(data, -EIO);
		return -EIO;
	}
	data->updates_active = false;
	data->suspend_blocked = false;
	data->system_suspend_in_progress = false;
	mutex_unlock(&data->queue_mutex);

	return 0;

resume_init_failed:
	epdc_recovery_dump(data, "resume-init-failed");
	epdc_finish_recovery(data, ret);
	return ret;
}
#else
#define mxc_epdc_fb_suspend	NULL
#define mxc_epdc_fb_resume	NULL
#endif

#ifdef CONFIG_PM
static int mxc_epdc_fb_runtime_suspend(struct device *dev)
{
	release_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "epdc busfreq high release.\n");

	return 0;
}

static int mxc_epdc_fb_runtime_resume(struct device *dev)
{
	request_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "epdc busfreq high request.\n");

	return 0;
}
#else
#define mxc_epdc_fb_runtime_suspend	NULL
#define mxc_epdc_fb_runtime_resume	NULL
#endif

static const struct dev_pm_ops mxc_epdc_fb_pm_ops = {
	SET_RUNTIME_PM_OPS(mxc_epdc_fb_runtime_suspend,
				mxc_epdc_fb_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(mxc_epdc_fb_suspend, mxc_epdc_fb_resume)
};

static void mxc_epdc_fb_shutdown(struct platform_device *pdev)
{
	struct mxc_epdc_fb_data *fb_data = platform_get_drvdata(pdev);

#ifdef CONFIG_PM_SLEEP
	int ret;

	/* The normal suspend path blocks new submissions, drains queued/LUT
	 * updates and powers the panel down in order. Cutting VCOM directly
	 * during the erase phase can permanently leave a white shutdown frame.
	 */
	ret = mxc_epdc_fb_suspend(&pdev->dev);
	if (!ret)
	    return;
	dev_warn(&pdev->dev, "shutdown EPDC drain failed (%d); forcing power off\n", ret);
	/* A failed drain must not leave userspace free to submit more frames. */
	mutex_lock(&fb_data->queue_mutex);
	fb_data->suspend_blocked = true;
	mutex_unlock(&fb_data->queue_mutex);
#endif

	/* Disable power to the EPD panel */
	if (regulator_is_enabled(fb_data->vcom_regulator))
		regulator_disable(fb_data->vcom_regulator);
	if (regulator_is_enabled(fb_data->display_regulator))
		regulator_disable(fb_data->display_regulator);

	/* Disable clocks to EPDC */
	clk_prepare_enable(fb_data->epdc_clk_axi);
	clk_prepare_enable(fb_data->epdc_clk_pix);
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_SET);
	clk_disable_unprepare(fb_data->epdc_clk_pix);
	clk_disable_unprepare(fb_data->epdc_clk_axi);

	/* turn off the V3p3 */
	if (regulator_is_enabled(fb_data->v3p3_regulator))
		regulator_disable(fb_data->v3p3_regulator);
}

static struct platform_driver mxc_epdc_fb_driver = {
	.probe = mxc_epdc_fb_probe,
	.remove = mxc_epdc_fb_remove,
	.shutdown = mxc_epdc_fb_shutdown,
	.driver = {
		   .name = "imx_epdc_fb",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(imx_epdc_dt_ids),
		   .pm = &mxc_epdc_fb_pm_ops,
		   },
};

/* Callback function triggered after PxP receives an EOF interrupt */
static void pxp_dma_done(void *arg)
{
	struct pxp_tx_desc *tx_desc = to_tx_desc(arg);
	struct dma_chan *chan = tx_desc->txd.chan;
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	struct mxc_epdc_fb_data *fb_data = pxp_chan->client;

	/*
	 * pxp_irq() frees the DMA descriptor immediately after this callback.
	 * Copy histogram status while the descriptor is still alive; reading
	 * fb_data->txd after completion is a use-after-free.
	 */
	fb_data->pxp_hist_status = tx_desc->hist_status;
	complete(&fb_data->pxp_tx_cmpl);
}

static bool chan_filter(struct dma_chan *chan, void *arg)
{
	if (imx_dma_is_pxp(chan))
		return true;
	else
		return false;
}

/* Function to request PXP DMA channel */
static int pxp_chan_init(struct mxc_epdc_fb_data *fb_data)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;

	/*
	 * Request a free channel
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);
	chan = dma_request_channel(mask, chan_filter, NULL);
	if (!chan) {
		dev_err(fb_data->dev, "Unsuccessfully received channel!!!!\n");
		return -EBUSY;
	}

	fb_data->pxp_chan = to_pxp_channel(chan);
	fb_data->pxp_debug_chan = &fb_data->pxp_chan->dma_chan;
	fb_data->pxp_chan->client = fb_data;

	init_completion(&fb_data->pxp_tx_cmpl);

	return 0;
}

/*
 * Function to call PxP DMA driver and send our latest FB update region
 * through the PxP and out to an intermediate buffer.
 * Note: This is a blocking call, so upon return the PxP tx should be complete.
 */
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region,
			      struct update_desc_list *source_desc)
{
	dma_cookie_t cookie;
	struct scatterlist *sg = fb_data->sg;
	struct dma_chan *dma_chan;
	struct pxp_tx_desc *desc;
	struct dma_async_tx_descriptor *txd;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &fb_data->pxp_conf.proc_data;
	int i, ret;
	int length;
	bool source_page_reused = false;

	dev_dbg(fb_data->dev, "Starting PxP Send Buffer\n");

	/* First, check to see that we have acquired a PxP Channel object */
	if (fb_data->pxp_chan == NULL) {
		/*
		 * PxP Channel has not yet been created and initialized,
		 * so let's go ahead and try
		 */
		ret = pxp_chan_init(fb_data);
		if (ret) {
			/*
			 * PxP channel init failed, and we can't use the
			 * PxP until the PxP DMA driver has loaded, so we abort
			 */
			dev_err(fb_data->dev, "PxP chan init failed\n");
			return -ENODEV;
		}
	}

	/*
	 * Init completion, so that we
	 * can be properly informed of the completion
	 * of the PxP task when it is done.
	 */
	init_completion(&fb_data->pxp_tx_cmpl);
	fb_data->pxp_hist_status = 0;

	dma_chan = &fb_data->pxp_chan->dma_chan;

	txd = dma_chan->device->device_prep_slave_sg(dma_chan, sg, 2,
						     DMA_TO_DEVICE,
						     DMA_PREP_INTERRUPT,
						     NULL);
	if (!txd) {
		dev_err(fb_data->info.device,
			"Error preparing a DMA transaction descriptor.\n");
		return -EIO;
	}

	txd->callback_param = txd;
	txd->callback = pxp_dma_done;

	/*
	 * Configure PxP for processing of new update region
	 * The rest of our config params were set up in
	 * probe() and should not need to be changed.
	 */
	pxp_conf->s0_param.width = src_width;
	pxp_conf->s0_param.height = src_height;
	proc_data->srect.top = update_region->top;
	proc_data->srect.left = update_region->left;
	proc_data->srect.width = update_region->width;
	proc_data->srect.height = update_region->height;

	/*
	 * Because only YUV/YCbCr image can be scaled, configure
	 * drect equivalent to srect, as such do not perform scaling.
	 */
	proc_data->drect.top = 0;
	proc_data->drect.left = 0;

	/* PXP expects rotation in terms of degrees */
	proc_data->rotate = fb_data->epdc_fb_var.rotate * 90;
	if (proc_data->rotate > 270)
		proc_data->rotate = 0;

	/* Just as V4L2 PXP, we should pass the rotated values to PXP */
	if ((proc_data->rotate == 90) || (proc_data->rotate == 270)) {
		proc_data->drect.width = proc_data->srect.height;
		proc_data->drect.height = proc_data->srect.width;
		pxp_conf->out_param.width = update_region->height;
		pxp_conf->out_param.height = update_region->width;
		pxp_conf->out_param.stride = update_region->height;
	} else {
		proc_data->drect.width = proc_data->srect.width;
		proc_data->drect.height = proc_data->srect.height;
		pxp_conf->out_param.width = update_region->width;
		pxp_conf->out_param.height = update_region->height;
		pxp_conf->out_param.stride = update_region->width;
	}

	/* For EPDC v2.0, we need output to be 64-bit
	 * aligned since EPDC stride does not work. */
	if (fb_data->rev <= 20)
		pxp_conf->out_param.stride = ALIGN(pxp_conf->out_param.stride, 8);


	desc = to_tx_desc(txd);
	length = desc->len;
	for (i = 0; i < length; i++) {
		if (i == 0) {/* S0 */
			memcpy(&desc->proc_data, proc_data, sizeof(struct pxp_proc_data));
			pxp_conf->s0_param.paddr = sg_dma_address(&sg[0]);
			memcpy(&desc->layer_param.s0_param, &pxp_conf->s0_param,
				sizeof(struct pxp_layer_param));
		} else if (i == 1) {
			pxp_conf->out_param.paddr = sg_dma_address(&sg[1]);
			memcpy(&desc->layer_param.out_param, &pxp_conf->out_param,
				sizeof(struct pxp_layer_param));
		}
		/* TODO: OverLay */

		desc = desc->next;
	}

	/* Sample the page generation immediately before the DMA transaction. */
	if (source_desc)
		source_page_reused =
			epdc_fb_page_generation_changed(fb_data, source_desc);

	/* Submitting our TX starts the PxP processing task */
	cookie = txd->tx_submit(txd);
	if (cookie < 0) {
		dev_err(fb_data->info.device, "Error sending FB through PxP\n");
		return -EIO;
	}
	if (source_page_reused)
		epdc_record_fb_page_reuse(fb_data, false);

	/* trigger ePxP */
	dma_async_issue_pending(dma_chan);

	return 0;
}

static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat)
{
	unsigned long ret;

	/*
	 * HZ/10 used to be a destructive timeout: the EPDC client released the
	 * DMA channel even though PxP could still be reading its source. A late
	 * IRQ then completed the old transaction against a recycled channel and
	 * could satisfy the next transaction's shared completion. Treat 100 ms as
	 * a slow-operation threshold only. Once a transaction is submitted, source
	 * and channel ownership are retained until the real PxP callback arrives.
	 * A genuine hardware hang therefore fails stopped instead of corrupting a
	 * later framebuffer/source buffer.
	 */
	ret = wait_for_completion_timeout(&fb_data->pxp_tx_cmpl, HZ / 10);
	if (!ret) {
		dev_warn_ratelimited(fb_data->info.device,
			"PxP operation exceeded 100 ms; waiting for real completion\n");
		wait_for_completion(&fb_data->pxp_tx_cmpl);
	}

	if ((fb_data->pxp_conf.proc_data.lut_transform & EPDC_FLAG_USE_CMAP) &&
		fb_data->pxp_conf.proc_data.lut_map_updated)
		fb_data->pxp_conf.proc_data.lut_map_updated = false;

	*hist_stat = fb_data->pxp_hist_status;
	dma_release_channel(&fb_data->pxp_chan->dma_chan);
	fb_data->pxp_chan = NULL;

	dev_dbg(fb_data->dev, "TX completed\n");

	return 0;
}

/*
 * Different dithering algorithm can be used. We chose
 * to implement Bill Atkinson's algorithm as an example
 * Thanks Bill Atkinson for his dithering algorithm.
 */

/*
 * Dithering algorithm implementation - Y8->Y1 version 1.0 for i.MX
 */
static void do_dithering_processing_Y1_v1_0(
		unsigned char *update_region_virt_ptr,
		dma_addr_t update_region_phys_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist)
{

	/* create a temp error distribution array */
	int bwPix;
	int y;
	int col;
	int *err_dist_l0, *err_dist_l1, *err_dist_l2, distrib_error;
	int width_3 = update_region->width + 3;
	char *y8buf;
	int x_offset = 0;

	/* prime a few elements the error distribution array */
	for (y = 0; y < update_region->height; y++) {
		/* Dithering the Y8 in sbuf to BW suitable for A2 waveform */
		err_dist_l0 = err_dist + (width_3) * (y % 3);
		err_dist_l1 = err_dist + (width_3) * ((y + 1) % 3);
		err_dist_l2 = err_dist + (width_3) * ((y + 2) % 3);

		y8buf = update_region_virt_ptr + x_offset;

		/* scan the line and convert the Y8 to BW */
		for (col = 1; col <= update_region->width; col++) {
			bwPix = *(err_dist_l0 + col) + *y8buf;

			if (bwPix >= 128) {
				*y8buf++ = 0xff;
				distrib_error = (bwPix - 255) >> 3;
			} else {
				*y8buf++ = 0;
				distrib_error = bwPix >> 3;
			}

			/* modify the error distribution buffer */
			*(err_dist_l0 + col + 2) += distrib_error;
			*(err_dist_l1 + col + 1) += distrib_error;
			*(err_dist_l0 + col + 1) += distrib_error;
			*(err_dist_l1 + col - 1) += distrib_error;
			*(err_dist_l1 + col) += distrib_error;
			*(err_dist_l2 + col) = distrib_error;
		}
		x_offset += update_region_stride;
	}

	flush_cache_all();
	outer_flush_range(update_region_phys_ptr, update_region_phys_ptr +
			update_region->height * update_region->width);
}

/*
 * Dithering algorithm implementation - Y8->Y4 version 1.0 for i.MX
 */

static void do_dithering_processing_Y4_v1_0(
		unsigned char *update_region_virt_ptr,
		dma_addr_t update_region_phys_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist)
{

	/* create a temp error distribution array */
	int gcPix;
	int y;
	int col;
	int *err_dist_l0, *err_dist_l1, *err_dist_l2, distrib_error;
	int width_3 = update_region->width + 3;
	char *y8buf;
	int x_offset = 0;

	/* prime a few elements the error distribution array */
	for (y = 0; y < update_region->height; y++) {
		/* Dithering the Y8 in sbuf to Y4 */
		err_dist_l0 = err_dist + (width_3) * (y % 3);
		err_dist_l1 = err_dist + (width_3) * ((y + 1) % 3);
		err_dist_l2 = err_dist + (width_3) * ((y + 2) % 3);

		y8buf = update_region_virt_ptr + x_offset;

		/* scan the line and convert the Y8 to Y4 */
		for (col = 1; col <= update_region->width; col++) {
			gcPix = *(err_dist_l0 + col) + *y8buf;

			if (gcPix > 255)
				gcPix = 255;
			else if (gcPix < 0)
				gcPix = 0;

			distrib_error = (*y8buf - (gcPix & 0xf0)) >> 3;

			*y8buf++ = gcPix & 0xf0;

			/* modify the error distribution buffer */
			*(err_dist_l0 + col + 2) += distrib_error;
			*(err_dist_l1 + col + 1) += distrib_error;
			*(err_dist_l0 + col + 1) += distrib_error;
			*(err_dist_l1 + col - 1) += distrib_error;
			*(err_dist_l1 + col) += distrib_error;
			*(err_dist_l2 + col) = distrib_error;
		}
		x_offset += update_region_stride;
	}

	flush_cache_all();
	outer_flush_range(update_region_phys_ptr, update_region_phys_ptr +
			update_region->height * update_region->width);
}

static int __init mxc_epdc_fb_init(void)
{
	return platform_driver_register(&mxc_epdc_fb_driver);
}
late_initcall(mxc_epdc_fb_init);

static void __exit mxc_epdc_fb_exit(void)
{
	platform_driver_unregister(&mxc_epdc_fb_driver);
}
module_exit(mxc_epdc_fb_exit);


MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC EPDC framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");
