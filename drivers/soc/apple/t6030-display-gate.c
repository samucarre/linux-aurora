// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T6030 internal display enable gate, and its T8122 counterpart
 *
 * The T6030 device tree describes the internal display (the DCP, its
 * mailbox, the two display DARTs and the display subsystem) with every node
 * disabled. Kernels that cannot take the display over from iBoot therefore
 * keep scanning out through the simple framebuffer, and all of them can
 * share one device tree.
 *
 * This gate enables those five nodes before platform devices are created.
 * It acts only if the boot loader has set apple,t6030-handoff = <1> on the
 * DCP and on the display subsystem, which it does after it has checked and
 * locked the inherited DART mappings. apple_t6030_display.enable=0 on the
 * kernel command line keeps the display on the boot framebuffer instead;
 * apple_t6030_display.dcpext=0 and apple_t6030_display.scanout=0 do the same
 * for the external display processors and their display DARTs (see
 * gate_dcpext() below). The targets are found through the DCP's and the
 * display subsystem's phandles, and each must still be disabled. Otherwise
 * nothing is changed.
 *
 * The DCP firmware needs the power management processor (PMP) running,
 * which iBoot leaves loaded but halted. Once the display nodes are enabled,
 * the gate also adds the PMP from a built-in overlay: its node, DART and
 * mailbox (disabled), and a PMP report node whose driver sets the display
 * and storage requests and starts the PMP when the display driver asks for
 * it. The PMP node needs this Mac's board and DRAM vendor ids and its PMP
 * tunables. A boot loader that copies them from the ADT into
 * /chosen/asahi,t6030-pmp gets the T6030 overlay with those values, on any
 * T6030 board; without them, only a J516S gets the PMP, from an overlay with
 * one J516S's values. Before any driver probes, the gate then
 *  - raises the minimum power state of the display domains (display
 *    subsystem, front end and DCP CPU) to "active", so that the PMP can
 *    never power the running DCP down;
 *  - keeps the PMP and PMS_SRAM domains on, as iBoot left them: the PTD the
 *    DCP firmware and the PMP share lives in that SRAM;
 *  - gives the PMP DART and mailbox the interrupt parent of the DCP mailbox.
 * Everything the PMP needs is checked before the display nodes are enabled.
 * Without PMP values for this Mac, or if a check fails or the PMP cannot be
 * added, the display nodes stay (or are put back) disabled and the display
 * stays on the boot framebuffer.
 *
 * What differs between SoCs is in struct gate_soc. On T8122 (M3) the gate
 * serves the same handoff, marked apple,t8122-handoff by the boot loader on
 * apple,t8122-dcp and apple,t8122-display-subsystem nodes. Its device tree
 * already describes the PMP, its DART, mailbox and report (disabled where it
 * must be), so the gate adds no overlay there: it checks that the PMP
 * topology is complete and unused, and then completes it (see
 * gate_reuse_resolve()). There is no T8122 fallback: the boot loader must pass
 * this Mac's PMP values in /chosen/asahi,t8122-pmp, or nothing is enabled. A
 * T8122 device tree without display nodes is normal and the gate says
 * nothing. apple_t8122_display.enable=0 has the same effect as the T6030
 * option.
 *
 * On T6031 (M3 Max) the gate serves the same handoff, marked
 * apple,t6031-handoff. Its default is scoped to J514C and J516C: there the native
 * display is on unless apple_t6031_display.enable=0; other T6031 boards
 * need apple_t6031_display.enable=1. The T6031 GPU starts its firmware only
 * after this native display is up, so enable=0 (or a display that never
 * comes up) keeps the GPU off for that boot too: it is not a display-only
 * switch. The default changes no check of the
 * handoff or the PMP below. The PMP values must come from the boot loader, in
 * /chosen/asahi,t6031-pmp; there is no fallback. Its PMP topology is
 * reused from the device tree, as on T8122. The T6031 display power
 * states nest the other way round (the DCP's DISP_FE under DISP_CPU under
 * DISP_SYS), and its PMP power states are in another PMGR block. On every
 * T6031 boot the gate also logs what the boot loader handed over, so that
 * a boot without the option already records what the next one needs. On a
 * J516C, apple_t6031_display.ext=1 as well also describes the HDMI port,
 * which the boot loader does not (see gate_t6031_ext_run()). On every T6031,
 * whatever the display switches, the gate also describes the PMU's RTC alarm
 * from the ADT if the device tree does not (see gate_t6031_rtc_run()).
 */

#define pr_fmt(fmt) fmt

#include <dt-bindings/phy/phy.h>
#include <linux/align.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kstrtox.h>
#include <linux/memblock.h>
#include <linux/minmax.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <video/nomodeset.h>

#ifdef CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST
#include <kunit/static_stub.h>
#endif

enum {
	GATE_DCP_DART,
	GATE_DISP0_DART,
	GATE_DCP_MBOX,
	GATE_DCP,
	GATE_DISPLAY,
	GATE_NR_NODES,
};

enum {
	PMP_PS_DISP_SYS,
	PMP_PS_DISP_FE,
	PMP_PS_DISP_CPU,
	PMP_PS_PMP,
	PMP_PS_PMS_SRAM,
	PMP_PS_NR,
};

static const char *const pmp_ps_labels[PMP_PS_NR] __initconst = {
	"disp_sys", "disp_fe", "disp_cpu", "pmp", "pms_sram",
};

#define PMGR_PS_ACTIVE	15

/*
 * Built-in overlays with the PMP, its DART and mailbox, and its report. The
 * T6030 one has no per-Mac values: the gate copies them from the node the
 * boot loader fills from this Mac's ADT. The J516S one carries the values
 * of one J516S, for a boot loader that passes none.
 */
extern const u8 __dtbo_t6030_pmp_begin[];
extern const u8 __dtbo_t6030_pmp_end[];
extern const u8 __dtbo_t6030_j516s_pmp_begin[];
extern const u8 __dtbo_t6030_j516s_pmp_end[];

/* This Mac's PMP values, from the boot loader, or NULL. */
static struct device_node *gate_pmp_values __initdata;

/*
 * The PMP firmware image the T6030 PMP support was brought up with, as the
 * J516S overlay names it. The report driver starts only the image the PMP
 * node names; values from the boot loader must name this one too.
 */
static const char gate_pmp_uuid[] __initconst = "2F4EB4C4-001B-3ACF-A9A0-68D8E42FC3A7";

/* The T8122 PMP image of the 14.x system firmware. */
static const char gate_t8122_pmp_uuid[] __initconst = "3B18C886-4C70-349B-B1C4-70F35DE2C5CD";

/*
 * The T6031 PMP image, as the J516C ADT names it (iop-pmp-nub, passed on by
 * the boot loader in /chosen/asahi,t6031-pmp). It is not the T6030 image.
 */
static const char gate_t6031_pmp_uuid[] __initconst = "07FF4C5B-CB2E-3DE6-95FF-3B107D19B7AB";

/*
 * apple_t6030_display.enable and apple_t8122_display.enable: the native
 * display is on unless one of them is 0.
 */
static bool gate_requested __initdata = true;

/*
 * apple_t6031_display.enable: -1 while it is not on the command line, else 0
 * or 1. Unset, the native display is on for the exact J514C/J516C + T6031 pairs.
 */
static int gate_t6031_switch __initdata = -1;

/* Only the current KUnit case may substitute its synthetic root identity. */
static bool __init gate_t6031_is_compatible(const char *compat)
{
#ifdef CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST
	KUNIT_STATIC_STUB_REDIRECT(gate_t6031_is_compatible, compat);
#endif
	return of_machine_is_compatible(compat);
}

static const char *__init gate_t6031_default_board(void)
{
	if (!gate_t6031_is_compatible("apple,t6031"))
		return NULL;
	if (gate_t6031_is_compatible("apple,j516c"))
		return "apple,j516c";
	if (gate_t6031_is_compatible("apple,j514c"))
		return "apple,j514c";
	return NULL;
}
/* The resolved request, set by gate_run() on a T6031. */
static bool gate_t6031_requested __initdata;

/* Whether the T6031 native display is requested: an explicit 0 or 1 wins. */
static bool __init gate_t6031_wanted(int sw, bool default_board)
{
	return sw < 0 ? default_board : sw;
}

/* T6034 has an independent owner disable and never admits an unmeasured board. */
static int gate_t6034_switch __initdata = -1;
static bool gate_t6034_requested __initdata;

static bool __init gate_t6034_default_board(void)
{
	return of_machine_is_compatible("apple,t6034") &&
	       of_machine_is_compatible("apple,j514m");
}

static bool __init gate_t6034_wanted(int sw)
{
	return gate_t6034_default_board() && gate_t6031_wanted(sw, true);
}

/* One SoC's internal display handoff, as the gate checks and completes it */
struct gate_soc {
	const char *machine;		/* root compatible */
	const char *prefix;		/* log prefix */
	const char *name;
	const char *dcp_compat;
	const char *display_compat;
	const char *marker;		/* set to <1> by the boot loader */
	const char *pwrstate_compat;	/* the PMGR power states */
	const char *dcp_full_name;	/* the DCP node the PMP overlay expects, under /soc */
	const char *pmp_values;		/* the boot loader's PMP values for this Mac */
	const char *pmp_uuid;		/* the PMP image the PMP support is written for */
	const char *report_compat;	/* the PMP report the overlay adds, or the reused one */
	const char *disp_entry;		/* its display request entry, which the DCP waits for */
	const u8 *pmp_dtbo, *pmp_dtbo_end;	/* NULL: no PMP description built in */
	/*
	 * Set: the device tree describes the PMP (pmp_compat), its DART, its
	 * mailbox and its report, and the gate completes those nodes instead of
	 * adding an overlay. ans_entry is the storage request the PMP must see
	 * when it starts; dispext_entry the external display request.
	 */
	bool reuse_pmp;
	const char *pmp_compat;
	const char *ans_entry;
	const char *dispext_entry;
	bool records;			/* PTD bits are soc-device records (T8122) */
	const char *fallback_board;	/* gets the fallback overlay without boot loader values */
	const u8 *fallback_dtbo, *fallback_dtbo_end;
	bool *requested;		/* the command line switch */
	const char *opt_in;		/* set: off unless this is on the command line */
	/* The DCP's power state and its parents, nearest first, as PMP_PS_*. */
	u8 ps_chain[3];
	bool pmp_ps_any_pmgr;		/* PMP and PMS_SRAM are not beside the DCP's power state */
	bool describe;			/* log the boot loader's handoff on every boot */
	const char *fw_uuids;		/* /chosen strings naming each firmware image */
	bool quiet_without_nodes;	/* no display nodes in the device tree is normal */
	bool dcpext;			/* the external display processors and their DARTs */
};

static const struct gate_soc gate_t6030 __initconst = {
	.machine = "apple,t6030",
	.prefix = "apple-t6030-display: ",
	.name = "T6030",
	.dcp_compat = "apple,t6030-dcp",
	.display_compat = "apple,t6030-display-subsystem",
	.marker = "apple,t6030-handoff",
	.pwrstate_compat = "apple,t6030-pmgr-pwrstate",
	.dcp_full_name = "dcp@28ec00000",
	.pmp_values = "/chosen/asahi,t6030-pmp",
	.pmp_uuid = gate_pmp_uuid,
	.report_compat = "apple,t6030-pmp-v2-report",
	.disp_entry = "report@7",
	.pmp_dtbo = __dtbo_t6030_pmp_begin,
	.pmp_dtbo_end = __dtbo_t6030_pmp_end,
	.fallback_board = "apple,j516s",
	.fallback_dtbo = __dtbo_t6030_j516s_pmp_begin,
	.fallback_dtbo_end = __dtbo_t6030_j516s_pmp_end,
	.requested = &gate_requested,
	.ps_chain = { PMP_PS_DISP_CPU, PMP_PS_DISP_FE, PMP_PS_DISP_SYS },
	.dcpext = true,
};

/*
 * T8122 (M3 MacBook Air): the same handoff and power states. The PMP and its
 * report are the device tree's own. The PMP values must come from the boot
 * loader: there is no built-in fallback.
 */
static const struct gate_soc gate_t8122 __initconst = {
	.machine = "apple,t8122",
	.prefix = "apple-t8122-display: ",
	.name = "T8122",
	.dcp_compat = "apple,t8122-dcp",
	.display_compat = "apple,t8122-display-subsystem",
	.marker = "apple,t8122-handoff",
	.pwrstate_compat = "apple,t8122-pmgr-pwrstate",
	.dcp_full_name = "dcp@28ec00000",
	.pmp_values = "/chosen/asahi,t8122-pmp",
	.pmp_uuid = gate_t8122_pmp_uuid,
	.report_compat = "apple,t8122-pmp-v2-report",
	.disp_entry = "report@7",
	.reuse_pmp = true,
	.pmp_compat = "apple,t8122-pmp-v2",
	.ans_entry = "report@11",
	.dispext_entry = "report@8",
	.records = true,
	.requested = &gate_requested,
	.ps_chain = { PMP_PS_DISP_CPU, PMP_PS_DISP_FE, PMP_PS_DISP_SYS },
	.quiet_without_nodes = true,
};

/*
 * T6031 (M3 Max): the T6030 handoff at the T6031 addresses, on by default on
 * J514C/J516C and off unless apple_t6031_display.enable=1 on other boards;
 * apple_t6031_display.enable=0 keeps it, and with it the GPU, off everywhere.
 * The PMP values must come from the boot
 * loader: there is no built-in fallback. The PMP image is T6031's own;
 * any other, the T6030 image included, is refused. The report driver checks
 * the image's SRAM layout before it starts the PMP. The internal display's
 * PMP request is bit 0x10.
 */
static const struct gate_soc gate_t6031 __initconst = {
	.machine = "apple,t6031",
	.prefix = "apple-t6031-display: ",
	.name = "T6031",
	.dcp_compat = "apple,t6031-dcp",
	.display_compat = "apple,t6031-display-subsystem",
	.marker = "apple,t6031-handoff",
	.pwrstate_compat = "apple,t6031-pmgr-pwrstate",
	.dcp_full_name = "dcp@386c00000",
	.pmp_values = "/chosen/asahi,t6031-pmp",
	.pmp_uuid = gate_t6031_pmp_uuid,
	.report_compat = "apple,t6031-pmp-v2-report",
	.disp_entry = "report@10",
	.reuse_pmp = true,
	.pmp_compat = "apple,t6031-pmp-v2",
	.ans_entry = "report@21",
	.requested = &gate_t6031_requested,
	.opt_in = "apple_t6031_display.enable=1",
	.ps_chain = { PMP_PS_DISP_FE, PMP_PS_DISP_CPU, PMP_PS_DISP_SYS },
	.pmp_ps_any_pmgr = true,
	.describe = true,
	.fw_uuids = "asahi,t6031-fw-uuids",
};

/* Fresh J514M ADT: shared PMP image/layout, separate root and handoff. */
static const struct gate_soc gate_t6034 __initconst = {
	.machine = "apple,t6034",
	.prefix = "apple-t6034-display: ",
	.name = "T6034",
	.dcp_compat = "apple,t6034-dcp",
	.display_compat = "apple,t6034-display-subsystem",
	.marker = "apple,t6034-handoff",
	.pwrstate_compat = "apple,t6031-pmgr-pwrstate",
	.dcp_full_name = "dcp@386c00000",
	.pmp_values = "/chosen/asahi,t6034-pmp",
	.pmp_uuid = gate_t6031_pmp_uuid,
	.report_compat = "apple,t6034-pmp-v2-report",
	.disp_entry = "report@10",
	.reuse_pmp = true,
	.pmp_compat = "apple,t6034-pmp-v2",
	.ans_entry = "report@21",
	.requested = &gate_t6034_requested,
	.ps_chain = { PMP_PS_DISP_FE, PMP_PS_DISP_CPU, PMP_PS_DISP_SYS },
	.pmp_ps_any_pmgr = true,
	.describe = true,
	.fw_uuids = "asahi,t6034-fw-uuids",
};

static const struct gate_soc *const gate_socs[] __initconst = {
	&gate_t6030,
	&gate_t8122,
	&gate_t6031,
	&gate_t6034,
};

/* The SoC this boot runs on, if the gate serves it. */
static const struct gate_soc *gate_soc __initdata;

#define gate_info(fmt, ...) pr_info("%s" fmt, gate_soc->prefix, ##__VA_ARGS__)
#define gate_warn(fmt, ...) pr_warn("%s" fmt, gate_soc->prefix, ##__VA_ARGS__)
#define gate_err(fmt, ...) pr_err("%s" fmt, gate_soc->prefix, ##__VA_ARGS__)

/* The boot loader's values the gate copies onto the PMP node. */
static bool __init gate_pmp_value_wanted(const struct property *prop)
{
	if (!strcmp(prop->name, "apple,board-id") ||
	    !strcmp(prop->name, "apple,dram-vendor-id") ||
	    !strcmp(prop->name, "apple,dram-capacity"))
		return prop->length == sizeof(u32);
	return strstarts(prop->name, "apple,tunable-");
}

/* Kept after a successful apply: the live tree now holds its properties. */
static struct of_changeset gate_cs;
static struct of_changeset gate_pmp_cs;
/* The PMP overlay, once applied; only the KUnit test removes it again. */
static int gate_pmp_ovcs __initdata;
/* gate_pmp_cs completed the device tree's own PMP; only the KUnit test reverts it. */
static bool gate_pmp_reused __initdata;

/* The device tree's own PMP topology, found by gate_reuse_resolve(). */
struct gate_reuse {
	struct device_node *report, *pmp, *dart, *mbox, *disp, *ans;
};

static struct gate_reuse gate_reuse __initdata;

static void __init gate_reuse_put(void)
{
	of_node_put(gate_reuse.ans);
	of_node_put(gate_reuse.disp);
	of_node_put(gate_reuse.mbox);
	of_node_put(gate_reuse.dart);
	of_node_put(gate_reuse.pmp);
	of_node_put(gate_reuse.report);
	memset(&gate_reuse, 0, sizeof(gate_reuse));
}

static int __init gate_setup(char *arg)
{
	return kstrtobool(arg, &gate_requested);
}
early_param("apple_t6030_display.enable", gate_setup);

static int __init gate_setup_t8122(char *arg)
{
	return gate_setup(arg);
}
early_param("apple_t8122_display.enable", gate_setup_t8122);

/*
 * Its own switch: the T6030 and T8122 options do not change it, nor it them.
 * A value kstrtobool() rejects sets nothing, so the board default stays.
 */
static int __init gate_setup_t6031(char *arg)
{
	bool on;
	int ret = kstrtobool(arg, &on);

	if (!ret)
		gate_t6031_switch = on;
	return ret;
}
early_param("apple_t6031_display.enable", gate_setup_t6031);

static int __init gate_setup_t6034(char *arg)
{
	bool on;
	int ret = kstrtobool(arg, &on);

	if (!ret)
		gate_t6034_switch = on;
	return ret;
}
early_param("apple_t6034_display.enable", gate_setup_t6034);

/* Returns the only node compatible with @compat, or NULL if there are none or several. */
static struct device_node *__init gate_find_one(const char *compat)
{
	struct device_node *np, *found = NULL;

	for_each_compatible_node(np, NULL, compat) {
		if (found) {
			of_node_put(np);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(np);
	}

	return found;
}

static bool __init gate_marked(const struct device_node *np)
{
	u32 val;

	return !of_property_read_u32(np, gate_soc->marker, &val) && val == 1;
}

static bool __init gate_disabled(const struct device_node *np)
{
	const char *status;

	return !of_property_read_string(np, "status", &status) &&
	       !strcmp(status, "disabled");
}

/*
 * Returns the node named by the single entry of @list in @np, if that entry
 * has @nargs cells (the first equal to @arg0) and the node is compatible
 * with @compat. Returns NULL otherwise.
 */
static struct device_node *__init gate_target(const struct device_node *np,
					      const char *list,
					      const char *cells_name,
					      int nargs, u32 arg0,
					      const char *compat)
{
	struct of_phandle_args args;

	if (of_count_phandle_with_args(np, list, cells_name) != 1)
		return NULL;
	if (of_parse_phandle_with_args(np, list, cells_name, 0, &args))
		return NULL;

	if (args.args_count != nargs || (nargs && args.args[0] != arg0) ||
	    !of_device_is_compatible(args.np, compat)) {
		of_node_put(args.np);
		return NULL;
	}

	return args.np;
}

static int __init gate_resolve(struct device_node **np)
{
	int i;

	struct device_node *any;

	if (gate_soc->quiet_without_nodes) {
		any = of_find_compatible_node(NULL, NULL, gate_soc->dcp_compat);
		if (!any)
			return -ENODEV;
		of_node_put(any);
	}

	np[GATE_DCP] = gate_find_one(gate_soc->dcp_compat);
	if (!np[GATE_DCP]) {
		gate_warn("not enabling: need exactly one %s node\n", gate_soc->dcp_compat);
		return -ENODEV;
	}

	np[GATE_DISPLAY] = gate_find_one(gate_soc->display_compat);
	if (!np[GATE_DISPLAY]) {
		gate_warn("not enabling: need exactly one %s node\n", gate_soc->display_compat);
		return -ENODEV;
	}

	if (!gate_marked(np[GATE_DCP]) || !gate_marked(np[GATE_DISPLAY])) {
		gate_warn("not enabling: the boot loader did not set %s\n", gate_soc->marker);
		return -EPERM;
	}

	np[GATE_DCP_DART] = gate_target(np[GATE_DCP], "iommus", "#iommu-cells",
					1, 5, "apple,t8110-dart");
	if (!np[GATE_DCP_DART]) {
		gate_warn("not enabling: %pOF iommus is not one apple,t8110-dart stream 5\n",
			np[GATE_DCP]);
		return -EINVAL;
	}

	np[GATE_DISP0_DART] = gate_target(np[GATE_DISPLAY], "iommus",
					  "#iommu-cells", 1, 0,
					  "apple,t8110-dart");
	if (!np[GATE_DISP0_DART]) {
		gate_warn("not enabling: %pOF iommus is not one apple,t8110-dart stream 0\n",
			np[GATE_DISPLAY]);
		return -EINVAL;
	}

	np[GATE_DCP_MBOX] = gate_target(np[GATE_DCP], "mboxes", "#mbox-cells",
					0, 0, "apple,asc-mailbox-v4");
	if (!np[GATE_DCP_MBOX]) {
		gate_warn("not enabling: %pOF mboxes is not one apple,asc-mailbox-v4\n",
			np[GATE_DCP]);
		return -EINVAL;
	}

	if (np[GATE_DCP_DART] == np[GATE_DISP0_DART]) {
		gate_warn("not enabling: the DCP and the display subsystem share %pOF\n",
			np[GATE_DCP_DART]);
		return -EINVAL;
	}

	for (i = 0; i < GATE_NR_NODES; i++) {
		if (!gate_disabled(np[i])) {
			gate_warn("not enabling: %pOF is not disabled\n", np[i]);
			return -EBUSY;
		}
	}

	return 0;
}

static int __init gate_apply(struct device_node **np)
{
	int i, ret = 0;

	of_changeset_init(&gate_cs);

	for (i = 0; i < GATE_NR_NODES; i++) {
		ret = of_changeset_update_prop_string(&gate_cs, np[i], "status",
						      "okay");
		if (ret)
			break;
	}
	if (!ret)
		ret = of_changeset_apply(&gate_cs);

	if (ret) {
		of_changeset_destroy(&gate_cs);
		gate_err("not enabling: changeset failed: %d\n", ret);
		return ret;
	}

	gate_info("enabled %pOF, %pOF, %pOF, %pOF and %pOF\n",
		np[GATE_DCP_DART], np[GATE_DISP0_DART], np[GATE_DCP_MBOX],
		np[GATE_DCP], np[GATE_DISPLAY]);

	return 0;
}

/* Disables the display nodes again when the PMP could not be added. */
static void __init gate_revert(void)
{
	int ret = of_changeset_revert(&gate_cs);

	if (ret) {
		gate_err("could not disable the display nodes again: %d\n", ret);
		return;
	}
	of_changeset_destroy(&gate_cs);
	gate_info("display nodes disabled again, display stays on the boot framebuffer\n");
}

/* The single power domain of @np, if it is one of the SoC's power states. */
static struct device_node *__init gate_ps_parent(struct device_node *np)
{
	return gate_target(np, "power-domains", "#power-domain-cells", 0, 0,
			   gate_soc->pwrstate_compat);
}

static bool __init gate_ps_is(const struct device_node *np, const char *label)
{
	const char *name;

	return np && of_device_is_compatible(np, gate_soc->pwrstate_compat) &&
	       !of_property_read_string(np, "label", &name) && !strcmp(name, label);
}

/* The only power state below @pmgr labelled @label. */
static struct device_node *__init gate_ps_child(struct device_node *pmgr, const char *label)
{
	struct device_node *child, *found = NULL;

	for_each_child_of_node(pmgr, child) {
		if (!gate_ps_is(child, label))
			continue;
		if (found) {
			of_node_put(child);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(child);
	}

	return found;
}

/* The only power state of this SoC labelled @label, in any PMGR block. */
static struct device_node *__init gate_ps_unique(const char *label)
{
	struct device_node *np, *found = NULL;

	for_each_compatible_node(np, NULL, gate_soc->pwrstate_compat) {
		if (!gate_ps_is(np, label))
			continue;
		if (found) {
			of_node_put(np);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(np);
	}

	return found;
}

/* The available report entry of @report named @full_name. */
static struct device_node *__init gate_report_entry(struct device_node *report,
						    const char *full_name)
{
	struct device_node *child;

	for_each_available_child_of_node(report, child) {
		if (!strcmp(of_node_full_name(child), full_name) &&
		    of_device_is_compatible(child, "apple,t6000-pmp-v2-report-entry"))
			return child;
	}
	return NULL;
}

/*
 * The PMP topology a device tree describes itself (T8122, T6031): the report,
 * linked to its PMP by apple,pmp; the PMP, disabled and with no values yet,
 * with one DART stream and one ASC mailbox; the display request entry, always
 * on and the DCP's apple,pmp-report; and the storage request entry. Nothing
 * here is changed. The report driver starts the PMP, and enables it, once the
 * display driver asks; the DART and the mailbox are left as described.
 */
static int __init gate_reuse_resolve(struct device_node **np)
{
	struct gate_reuse *r = &gate_reuse;
	struct device_node *other, *link;
	bool single;

	r->report = gate_find_one(gate_soc->report_compat);
	if (!r->report || !of_device_is_available(r->report)) {
		gate_warn("PMP not added: need exactly one available %s node\n",
			  gate_soc->report_compat);
		return -ENODEV;
	}
	r->pmp = of_parse_phandle(r->report, "apple,pmp", 0);
	other = gate_find_one(gate_soc->pmp_compat);
	single = r->pmp && r->pmp == other;
	of_node_put(other);
	if (!single || !of_device_is_compatible(r->pmp, "apple,t6000-pmp-v2") ||
	    !gate_disabled(r->pmp) || !r->pmp->phandle) {
		gate_warn("PMP not added: %pOF apple,pmp is not the only %s node, disabled\n",
			  r->report, gate_soc->pmp_compat);
		return -ENODEV;
	}
	if (of_property_present(r->pmp, "apple,board-id") ||
	    of_property_present(r->pmp, "apple,dram-vendor-id") ||
	    of_property_present(r->pmp, "apple,tunable-uuid")) {
		gate_warn("PMP not added: %pOF already has PMP values\n", r->pmp);
		return -EEXIST;
	}
	r->dart = gate_target(r->pmp, "iommus", "#iommu-cells", 1, 0, "apple,t8110-dart");
	r->mbox = gate_target(r->pmp, "mboxes", "#mbox-cells", 0, 0, "apple,asc-mailbox-v4");
	if (!r->dart || !r->mbox) {
		gate_warn("PMP not added: %pOF is not on one apple,t8110-dart stream 0 and one apple,asc-mailbox-v4\n",
			  r->pmp);
		return -EINVAL;
	}

	r->disp = gate_report_entry(r->report, gate_soc->disp_entry);
	if (!r->disp || !r->disp->phandle ||
	    !of_property_read_bool(r->disp, "apple,always-on") ||
	    of_property_read_bool(r->disp, "apple,no-ack")) {
		gate_warn("PMP not added: %pOF has no acknowledged always-on display request %s\n",
			  r->report, gate_soc->disp_entry);
		return -ENODEV;
	}
	link = of_parse_phandle(np[GATE_DCP], "apple,pmp-report", 0);
	of_node_put(link);
	if (link != r->disp) {
		gate_warn("PMP not added: %pOF apple,pmp-report is not %pOF\n",
			  np[GATE_DCP], r->disp);
		return -EINVAL;
	}
	r->ans = gate_report_entry(r->report, gate_soc->ans_entry);
	if (!r->ans) {
		gate_warn("PMP not added: %pOF has no storage request %s\n",
			  r->report, gate_soc->ans_entry);
		return -ENODEV;
	}
	return 0;
}

/*
 * The display domains are the DCP's power domain and its parents, in the
 * SoC's order; the PMP and PMS_SRAM domains are found by label beside them,
 * or anywhere on a SoC that has them in another PMGR block.
 */
static int __init gate_pmp_resolve(struct device_node **np, struct device_node **ps,
				   struct device_node **aic)
{
	const u8 *chain = gate_soc->ps_chain;
	struct device_node *pmgr, *other;
	int i;

	if (!gate_soc->reuse_pmp && !gate_soc->pmp_dtbo) {
		gate_warn("PMP not added: no %s PMP description is built in\n", gate_soc->name);
		return -ENODEV;
	}

	gate_pmp_values = of_find_node_by_path(gate_soc->pmp_values);
	if (gate_pmp_values) {
		const char *uuid = NULL;
		u32 v;

		if (of_property_read_u32(gate_pmp_values, "apple,board-id", &v) ||
		    of_property_read_u32(gate_pmp_values, "apple,dram-vendor-id", &v) ||
		    of_property_read_string(gate_pmp_values, "apple,tunable-uuid", &uuid)) {
			gate_warn("PMP not added: %pOF is incomplete\n", gate_pmp_values);
			of_node_put(gate_pmp_values);
			gate_pmp_values = NULL;
			return -EINVAL;
		}
		if (strcmp(uuid, gate_soc->pmp_uuid)) {
			gate_warn("PMP not added: this Mac's PMP firmware image is %s, not %s\n",
				uuid, gate_soc->pmp_uuid);
			of_node_put(gate_pmp_values);
			gate_pmp_values = NULL;
			return -EINVAL;
		}
	}
	if (!gate_pmp_values && (!gate_soc->fallback_board ||
				 !of_machine_is_compatible(gate_soc->fallback_board))) {
		gate_warn("PMP not added: the boot loader passed no PMP values for this Mac\n");
		return -ENODEV;
	}

	if (gate_soc->reuse_pmp) {
		int ret = gate_reuse_resolve(np);

		if (ret)
			return ret;
	} else {
		other = of_find_compatible_node(NULL, NULL, "apple,t6000-pmp-v2");
		if (!other)
			other = of_find_compatible_node(NULL, NULL, gate_soc->report_compat);
		if (other) {
			gate_warn("PMP not added: %pOF already exists\n", other);
			of_node_put(other);
			return -EEXIST;
		}

		/* The overlay's DCP fragment names this path. */
		if (strcmp(of_node_full_name(np[GATE_DCP]), gate_soc->dcp_full_name) ||
		    !of_node_name_eq(np[GATE_DCP]->parent, "soc") ||
		    !of_node_is_root(np[GATE_DCP]->parent->parent)) {
			gate_warn("PMP not added: the DCP is %pOF, not /soc/%s\n",
				  np[GATE_DCP], gate_soc->dcp_full_name);
			return -EINVAL;
		}
	}

	ps[chain[0]] = gate_ps_parent(np[GATE_DCP]);
	if (ps[chain[0]])
		ps[chain[1]] = gate_ps_parent(ps[chain[0]]);
	if (ps[chain[1]])
		ps[chain[2]] = gate_ps_parent(ps[chain[1]]);
	if (gate_soc->pmp_ps_any_pmgr) {
		ps[PMP_PS_PMP] = gate_ps_unique("pmp");
		ps[PMP_PS_PMS_SRAM] = gate_ps_unique("pms_sram");
	} else {
		pmgr = of_get_parent(ps[PMP_PS_DISP_CPU]);
		if (pmgr) {
			ps[PMP_PS_PMP] = gate_ps_child(pmgr, "pmp");
			ps[PMP_PS_PMS_SRAM] = gate_ps_child(pmgr, "pms_sram");
			of_node_put(pmgr);
		}
	}
	for (i = 0; i < PMP_PS_NR; i++) {
		if (!gate_ps_is(ps[i], pmp_ps_labels[i])) {
			gate_warn("PMP not added: no %s power state where expected\n",
				pmp_ps_labels[i]);
			return -ENODEV;
		}
	}
	for (i = PMP_PS_DISP_SYS; i <= PMP_PS_DISP_CPU; i++) {
		if (!of_property_read_bool(ps[i], "apple,inherited-on")) {
			gate_warn("PMP not added: %pOF is not apple,inherited-on\n", ps[i]);
			return -EINVAL;
		}
	}

	/* Only the overlay's nodes need it: the reused ones have their own. */
	if (gate_soc->reuse_pmp)
		return 0;
	*aic = of_parse_phandle(np[GATE_DCP_MBOX], "interrupt-parent", 0);
	if (!*aic || !of_property_read_bool(*aic, "interrupt-controller")) {
		gate_warn("PMP not added: %pOF has no interrupt parent\n", np[GATE_DCP_MBOX]);
		return -EINVAL;
	}

	return 0;
}

/* Sets the u32 property @name of @np, adding it if it is missing. */
static int __init gate_set_u32(struct of_changeset *cs, struct device_node *np,
			       const char *name, u32 val)
{
	struct property *prop;
	__be32 *value;

	/* Kept for good: the live tree refers to it once applied. */
	prop = kzalloc_obj(*prop);
	value = kmalloc_obj(*value);
	if (!prop || !value) {
		kfree(prop);
		kfree(value);
		return -ENOMEM;
	}
	*value = cpu_to_be32(val);
	prop->name = (char *)name;
	prop->length = sizeof(*value);
	prop->value = value;

	return of_changeset_update_property(cs, np, prop);
}

/*
 * Adds the boot loader's PMP values (gate_pmp_value_wanted()) to @pmp, which
 * the T6030 overlay leaves without them. Returns the number added.
 */
static int __init gate_pmp_copy_values(struct of_changeset *cs, struct device_node *pmp)
{
	struct property *src, *prop;
	int n = 0, ret;

	for_each_property_of_node(gate_pmp_values, src) {
		if (!gate_pmp_value_wanted(src))
			continue;
		/* Kept for good: the live tree refers to it once applied. */
		prop = kzalloc_obj(*prop);
		if (!prop)
			return -ENOMEM;
		prop->name = kstrdup(src->name, GFP_KERNEL);
		prop->value = kmemdup(src->value, src->length, GFP_KERNEL);
		prop->length = src->length;
		if (!prop->name || (src->length && !prop->value)) {
			kfree(prop->name);
			kfree(prop->value);
			kfree(prop);
			return -ENOMEM;
		}
		ret = of_changeset_add_property(cs, pmp, prop);
		if (ret) {
			kfree(prop->name);
			kfree(prop->value);
			kfree(prop);
			return ret;
		}
		n++;
	}
	return n;
}

static int __init gate_gpu_pmp_link(struct of_changeset *cs, struct device_node *pmp)
{
	struct device_node *gpu, *linked;
	bool t6030 = IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK) && gate_soc == &gate_t6030;
	int ret = 0;

	if (gate_soc != &gate_t8122 && !t6030)
		return 0;
	gpu = t6030 ? gate_find_one("apple,agx-t6030") :
		of_find_compatible_node(NULL, NULL, "apple,agx-t8122");
	if (!gpu || !of_device_is_available(gpu))
		goto out;
	/* Do not replace a malformed or unresolved pre-existing GPU supplier. */
	if (t6030 && of_property_present(gpu, "apple,pmp") &&
	    of_property_count_u32_elems(gpu, "apple,pmp") != 1) {
		ret = -EINVAL;
		goto out;
	}
	linked = of_parse_phandle(gpu, "apple,pmp", 0);
	if (!pmp || !pmp->phandle || (linked && linked != pmp) ||
	    (t6030 && of_property_present(gpu, "apple,pmp") && !linked))
		ret = -EINVAL;
	else if (!linked)
		ret = of_changeset_add_prop_u32(cs, gpu, "apple,pmp", pmp->phandle);
	of_node_put(linked);
out:
	of_node_put(gpu);
	return ret;
}

/* Adds the boolean @name to @np unless it is there already. */
static int __init gate_add_bool(struct of_changeset *cs, struct device_node *np,
				const char *name)
{
	if (of_property_read_bool(np, name))
		return 0;
	return of_changeset_add_prop_bool(cs, np, name);
}

/*
 * Completes the device tree's own PMP topology (gate_reuse_resolve()) in one
 * changeset, as the overlay path completes the overlay's:
 *  - the display request is requested by soc-device record where the PTD
 *    is indexed that way (T8122);
 *  - the storage request is always on, and not acknowledged, as the PMP
 *    keeps a block up only if its request was set when it started;
 *  - the T8122 GPU, if available, is linked to the PMP;
 *  - the display domains' floor, and the PMP and PMS_SRAM domains kept on;
 *  - this Mac's PMP values on the PMP node.
 * The PMP stays disabled: the report driver enables it once it starts it.
 */
static int __init gate_reuse_apply(struct device_node **ps)
{
	struct gate_reuse *r = &gate_reuse;
	int ret = 0, i, values = 0;

	of_changeset_init(&gate_pmp_cs);
	if (gate_soc->records)
		ret = gate_add_bool(&gate_pmp_cs, r->disp, "apple,soc-device-record");
	if (!ret)
		ret = gate_add_bool(&gate_pmp_cs, r->ans, "apple,always-on");
	/* The PMP does not acknowledge storage requests. */
	if (!ret)
		ret = gate_add_bool(&gate_pmp_cs, r->ans, "apple,no-ack");
	/* The current14 T8122 GPU needs an inner AGX vote from this Mac's PMP. */
	if (!ret)
		ret = gate_gpu_pmp_link(&gate_pmp_cs, r->pmp);
	/* The display domains' floor, applied by the PMGR driver when it probes. */
	for (i = PMP_PS_DISP_SYS; i <= PMP_PS_DISP_CPU && !ret; i++)
		ret = gate_set_u32(&gate_pmp_cs, ps[i], "apple,min-state", PMGR_PS_ACTIVE);
	/* Linux would otherwise power these off as unused. */
	for (i = PMP_PS_PMP; i <= PMP_PS_PMS_SRAM && !ret; i++)
		ret = gate_add_bool(&gate_pmp_cs, ps[i], "apple,always-on");
	if (!ret) {
		values = gate_pmp_copy_values(&gate_pmp_cs, r->pmp);
		ret = values < 0 ? values : 0;
	}
	if (!ret)
		ret = of_changeset_apply(&gate_pmp_cs);
	if (ret) {
		of_changeset_destroy(&gate_pmp_cs);
		gate_err("PMP not added: changeset failed: %d\n", ret);
		return ret;
	}
	gate_pmp_reused = true;

	gate_info("PMP: this Mac's %d values from %pOF on %pOF\n", values, gate_pmp_values,
		  r->pmp);
	gate_info("completed %pOF with its display (%pOF) and storage (%pOF) requests; minimum power state of %s, %s and %s raised to active; %s and %s kept on\n",
		  r->report, r->disp, r->ans,
		  pmp_ps_labels[PMP_PS_DISP_SYS], pmp_ps_labels[PMP_PS_DISP_FE],
		  pmp_ps_labels[PMP_PS_DISP_CPU], pmp_ps_labels[PMP_PS_PMP],
		  pmp_ps_labels[PMP_PS_PMS_SRAM]);
	return 0;
}

static int __init gate_pmp_apply(struct device_node *dcp, struct device_node **ps,
				 struct device_node *aic)
{
	const u8 *dtbo = gate_pmp_values ? gate_soc->pmp_dtbo : gate_soc->fallback_dtbo;
	const size_t size = gate_pmp_values ?
		gate_soc->pmp_dtbo_end - gate_soc->pmp_dtbo :
		gate_soc->fallback_dtbo_end - gate_soc->fallback_dtbo;
	struct device_node *report, *disp = NULL, *pmp = NULL, *dart = NULL, *mbox = NULL;
	int ovcs_id = 0, ret, i, values = 0;

	if (gate_soc->reuse_pmp)
		return gate_reuse_apply(ps);

	ret = of_overlay_fdt_apply(dtbo, size, &ovcs_id, NULL);
	if (ret) {
		gate_err("PMP not added: overlay failed: %d\n", ret);
		return ret;
	}

	report = of_find_compatible_node(NULL, NULL, gate_soc->report_compat);
	if (report) {
		struct device_node *child;

		pmp = of_parse_phandle(report, "apple,pmp", 0);
		/* The display request entry: DISP is PMP device 7 (0x10 on T6031). */
		for_each_child_of_node(report, child) {
			if (!strcmp(of_node_full_name(child), gate_soc->disp_entry)) {
				disp = child;
				break;
			}
		}
	}
	if (pmp) {
		dart = of_parse_phandle(pmp, "iommus", 0);
		mbox = of_parse_phandle(pmp, "mboxes", 0);
	}

	of_changeset_init(&gate_pmp_cs);
	ret = dart && mbox && disp && disp->phandle ? 0 : -ENODEV;
	/* The DCP driver starts once the PMP acknowledges this request. */
	if (!ret)
		ret = of_changeset_add_prop_u32(&gate_pmp_cs, dcp, "apple,pmp-report",
						disp->phandle);
	/* T8122 and an opted-in T6030 GPU need this Mac's PMP AGX vote. */
	if (!ret)
		ret = gate_gpu_pmp_link(&gate_pmp_cs, pmp);
	/* The DART and mailbox interrupt parent lives in the base tree. */
	if (!ret)
		ret = of_changeset_add_prop_u32(&gate_pmp_cs, dart, "interrupt-parent",
						aic->phandle);
	if (!ret)
		ret = of_changeset_add_prop_u32(&gate_pmp_cs, mbox, "interrupt-parent",
						aic->phandle);
	/* The display domains' floor, applied by the PMGR driver when it probes. */
	for (i = PMP_PS_DISP_SYS; i <= PMP_PS_DISP_CPU && !ret; i++)
		ret = gate_set_u32(&gate_pmp_cs, ps[i], "apple,min-state", PMGR_PS_ACTIVE);
	/* Linux would otherwise power these off as unused. */
	for (i = PMP_PS_PMP; i <= PMP_PS_PMS_SRAM && !ret; i++) {
		if (!of_property_read_bool(ps[i], "apple,always-on"))
			ret = of_changeset_add_prop_bool(&gate_pmp_cs, ps[i], "apple,always-on");
	}
	if (!ret && gate_pmp_values) {
		values = gate_pmp_copy_values(&gate_pmp_cs, pmp);
		ret = values < 0 ? values : 0;
	}
	if (!ret)
		ret = of_changeset_apply(&gate_pmp_cs);

	of_node_put(mbox);
	of_node_put(dart);
	of_node_put(pmp);
	of_node_put(disp);
	of_node_put(report);

	if (ret) {
		of_changeset_destroy(&gate_pmp_cs);
		of_overlay_remove(&ovcs_id);
		gate_err("PMP not added: changeset failed: %d\n", ret);
		return ret;
	}
	gate_pmp_ovcs = ovcs_id;

	if (gate_pmp_values)
		gate_info("PMP: this Mac's %d values from %pOF\n", values, gate_pmp_values);
	else
		gate_info("PMP: the built-in J516S values; the boot loader passed none\n");
	gate_info("added the PMP and its display and storage report; minimum power state of %s, %s and %s raised to active; %s and %s kept on\n",
		pmp_ps_labels[PMP_PS_DISP_SYS], pmp_ps_labels[PMP_PS_DISP_FE],
		pmp_ps_labels[PMP_PS_DISP_CPU], pmp_ps_labels[PMP_PS_PMP],
		pmp_ps_labels[PMP_PS_PMS_SRAM]);
	return 0;
}

/*
 * External display processors (dcpext0, dcpext1, ...). The boot loader hands
 * each one over separately: its memory (apple,t6030-dcpext-memory-ready), and
 * its display DART with a scanout child that names it
 * (apple,t6030-dispext-handoff). Nothing here touches a processor's CPU: the
 * display driver starts the firmware when a display is first attached.
 *
 * Each processor ends in one of three states:
 *  - native (the default): the processor, its DART, mailbox, PMP request and
 *    Type-C routes are enabled; its display DART is added to the display
 *    subsystem's iommus, so every framebuffer is mapped for this pipe as on
 *    M1/M2, and a piodma child is added for the DART's PIODMA stream 4. The
 *    processor then joins the main DRM device.
 *  - manual (apple_t6030_display.dcpext_manual=1): the earlier diagnostic
 *    path, with explicit start and a separately mapped scanout buffer.
 *  - refused: everything stays disabled, and so do its Type-C routes, so the
 *    USB-C ports never wait for a display mode switch nobody registers.
 * apple_t6030_display.dcpext=0 refuses every processor;
 * apple_t6030_display.scanout=0 refuses the display DART, which the native
 * path needs.
 */
static bool gate_dcpext_requested __initdata = true;

static int __init gate_dcpext_setup(char *arg)
{
	return kstrtobool(arg, &gate_dcpext_requested);
}
early_param("apple_t6030_display.dcpext", gate_dcpext_setup);

static bool gate_scanout_requested __initdata = true;

static int __init gate_scanout_setup(char *arg)
{
	return kstrtobool(arg, &gate_scanout_requested);
}
early_param("apple_t6030_display.scanout", gate_scanout_setup);

static bool gate_dcpext_manual __initdata;

static int __init gate_dcpext_manual_setup(char *arg)
{
	return kstrtobool(arg, &gate_dcpext_manual);
}
early_param("apple_t6030_display.dcpext_manual", gate_dcpext_manual_setup);

#define GATE_DCPEXT_MAX		4

/* One external display processor and what the gate found for it. */
struct gate_ext {
	struct device_node *dcp, *dart, *mbox;
	struct device_node *cpu, *fe, *sys;
	struct device_node *entry;	/* its PMP request, pmp-dispextN */
	struct device_node *scanout;	/* the boot loader's scanout child */
	struct device_node *disp_dart;	/* the display DART that child names */
	unsigned int index;		/* N of dispextN */
	bool memory;			/* the memory handoff is present */
	bool scanout_ok;		/* the display DART handoff checked out */
};

static void __init gate_ext_put(struct gate_ext *ext)
{
	of_node_put(ext->disp_dart);
	of_node_put(ext->scanout);
	of_node_put(ext->entry);
	of_node_put(ext->sys);
	of_node_put(ext->fe);
	of_node_put(ext->cpu);
	of_node_put(ext->mbox);
	of_node_put(ext->dart);
	of_node_put(ext->dcp);
}

/*
 * True if @ps is the power state labelled "dispext<N>_<suffix>". The first
 * call sets *@index (UINT_MAX on entry); later calls must name the same N.
 */
static bool __init gate_ps_dispext(const struct device_node *ps, const char *suffix,
				   unsigned int *index)
{
	const char *label, *p;
	unsigned int n = 0;

	if (!ps || !of_device_is_compatible(ps, gate_soc->pwrstate_compat) ||
	    of_property_read_string(ps, "label", &label) || !strstarts(label, "dispext"))
		return false;
	p = label + strlen("dispext");
	if (*p < '0' || *p > '9')
		return false;
	while (*p >= '0' && *p <= '9' && n < GATE_DCPEXT_MAX)
		n = n * 10 + *p++ - '0';
	if (n >= GATE_DCPEXT_MAX || *p++ != '_' || strcmp(p, suffix))
		return false;
	if (*index != UINT_MAX && *index != n)
		return false;
	*index = n;
	return true;
}

/* The PMP request entry labelled pmp-dispext<index>. */
static struct device_node *__init gate_pmp_dispext_entry(unsigned int index)
{
	struct device_node *report, *child, *found = NULL;
	char want[16];
	const char *label;

	report = gate_find_one(gate_soc->report_compat);
	if (!report || !of_device_is_available(report)) {
		of_node_put(report);
		return NULL;
	}
	snprintf(want, sizeof(want), "pmp-dispext%u", index);
	for_each_child_of_node(report, child) {
		if (!of_property_read_string(child, "label", &label) && !strcmp(label, want)) {
			found = child;
			break;
		}
	}
	of_node_put(report);
	return found;
}

/*
 * A no-map DT reservation is recorded by the reserved-memory parser, but
 * need not appear in memblock.reserved: no-map marks memblock.memory, and
 * the bootloader's carveouts can be outside the usable-memory list entirely.
 * memblock_is_region_reserved() only checks intersection, not full coverage.
 */
static bool __init gate_scanout_table_reserved(struct device_node *region,
					     const struct resource *table)
{
	struct reserved_mem *rmem;
	struct memblock_region *m;
	u64 size;

	if (!region || !region->parent ||
	    !of_node_name_eq(region->parent, "reserved-memory") ||
	    !of_node_is_root(region->parent->parent) ||
	    !of_device_is_available(region) ||
	    !of_property_read_bool(region, "no-map") ||
	    of_property_read_bool(region, "reusable") || table->end < table->start)
		return false;
	size = resource_size(table);
	if (size < SZ_16K || size > SZ_1M || !IS_ALIGNED(table->start, SZ_16K) ||
	    !IS_ALIGNED(size, SZ_16K))
		return false;
	rmem = of_reserved_mem_lookup(region);
	if (!rmem || rmem->base != table->start || rmem->size != size)
		return false;
	/* Reject even partially allocatable or linearly mapped RAM. */
	for_each_mem_region(m) {
		if (m->size && m->base <= table->end &&
		    (table->start < m->base || table->start - m->base < m->size) &&
		    !memblock_is_nomap(m))
			return false;
	}
	return true;
}

/*
 * Read-only check of the display DART the boot loader handed over with
 * @ext's scanout child, before Linux may attach to it: locked, translating,
 * with empty stream 0 and stream 4 roots in reserved tables, and powered by
 * the processor's own CPU domain.
 */
static int __init gate_dispext_check(struct gate_ext *ext)
{
	struct device_node *dart = NULL, *region = NULL;
	struct resource regs, table, first_table = {};
	struct of_phandle_args spec;
	void __iomem *mmio = NULL;
	void *root;
	u32 state[6], marker;
	u64 phys;
	int i, ret = -EINVAL;

	ext->scanout = of_get_child_by_name(ext->dcp, "scanout");
	if (!ext->scanout || !gate_disabled(ext->scanout) ||
	    !of_device_is_compatible(ext->scanout, "apple,t6030-dispext-scanout") ||
	    of_property_read_u32(ext->scanout, "apple,t6030-dispext-handoff", &marker) ||
	    marker != 1)
		return -ENODEV;
	if (of_count_phandle_with_args(ext->scanout, "iommus", "#iommu-cells") != 1 ||
	    of_parse_phandle_with_args(ext->scanout, "iommus", "#iommu-cells", 0, &spec))
		return -EINVAL;
	dart = spec.np;
	if (spec.args_count != 1 || spec.args[0] != 0 || !gate_disabled(dart) ||
	    !dart->phandle || dart == ext->dart ||
	    !of_device_is_compatible(dart, "apple,t8110-dart") ||
	    of_property_read_u32(dart, "apple,t6030-dispext-handoff", &marker) || marker != 1 ||
	    of_property_count_u32_elems(dart, "apple,inherited-dart-state") != 6 ||
	    of_property_read_u32_array(dart, "apple,inherited-dart-state", state, 6) ||
	    of_count_phandle_with_args(dart, "memory-region", NULL) != 2 ||
	    of_property_match_string(dart, "memory-region-names", "sid0-page-tables") != 0 ||
	    of_property_match_string(dart, "memory-region-names", "sid4-page-tables") != 1 ||
	    state[0] != 0 || state[3] != 4 ||
	    of_address_to_resource(dart, 0, &regs) || resource_size(&regs) != SZ_16K)
		goto out;
	/* The DART belongs to this processor's pipe: it shares its CPU domain. */
	region = gate_ps_parent(dart);
	if (region != ext->cpu)
		goto out;
	of_node_put(region);
	region = NULL;
	mmio = ioremap(regs.start, resource_size(&regs));
	if (!mmio || !(readl(mmio + 0x200) & BIT(0)))
		goto out;
	for (i = 0; i < 2; i++) {
		u32 sid = state[3 * i], tcr = state[3 * i + 1], ttbr = state[3 * i + 2];

		/* A valid SID0 root must not turn a later SID4 refusal into success. */
		ret = -EINVAL;
		if ((tcr & (BIT(0) | BIT(1) | BIT(3))) != BIT(0) || !(ttbr & BIT(0)) ||
		    (ttbr & ~(GENMASK(29, 2) | BIT(0))) ||
		    readl(mmio + 0x1000 + 4 * sid) != tcr ||
		    readl(mmio + 0x1400 + 4 * sid) != ttbr)
			goto out;
		/* T8110: address bits29:2, 16KiB page shift (42-bit output PA). */
		phys = (u64)(ttbr & GENMASK(29, 2)) << 12;
		region = of_parse_phandle(dart, "memory-region", i);
		if (!region ||
		    of_address_to_resource(region, 0, &table) ||
		    !gate_scanout_table_reserved(region, &table) ||
		    phys < table.start || phys > table.end - SZ_16K + 1 ||
		    (i && table.start <= first_table.end && first_table.start <= table.end))
			goto out;
		if (!i)
			first_table = table;
		/* arch_initcall runs after vmalloc initialization. On arm64,
		 * MEMREMAP_WB uses ioremap_cache for no-map/outside-usable RAM;
		 * the validation above excludes an allocatable/direct-map alias.
		 */
		root = memremap(phys, SZ_16K, MEMREMAP_WB);
		if (!root)
			goto out;
		ret = memchr_inv(root, 0, SZ_16K) ? -EBUSY : 0;
		memunmap(root);
		if (ret)
			goto out;
		of_node_put(region);
		region = NULL;
	}
	ext->disp_dart = of_node_get(dart);
	ext->scanout_ok = true;
out:
	if (mmio)
		iounmap(mmio);
	of_node_put(region);
	of_node_put(dart);
	return ret;
}

/* Everything @ext's processor needs, checked without changing anything. */
static int __init gate_ext_resolve(struct gate_ext *ext)
{
	struct device_node *domain;
	u32 ready, id;
	int i;

	ext->index = UINT_MAX;
	ext->memory = !of_property_read_u32(ext->dcp, "apple,t6030-dcpext-memory-ready", &ready) &&
		      ready == 1 && of_property_present(ext->dcp, "memory-region");
	if (!ext->memory)
		return -ENODEV;
	ext->dart = gate_target(ext->dcp, "iommus", "#iommu-cells", 1, 5, "apple,t8110-dart");
	ext->mbox = gate_target(ext->dcp, "mboxes", "#mbox-cells", 0, 0, "apple,asc-mailbox-v4");
	if (!ext->dart || !ext->mbox || !gate_disabled(ext->dcp) ||
	    !gate_disabled(ext->dart) || !gate_disabled(ext->mbox))
		return -EINVAL;

	/* Do not enable a CPU whose power ownership is not fully described. */
	ext->cpu = gate_ps_parent(ext->dcp);
	if (!gate_ps_dispext(ext->cpu, "cpu", &ext->index))
		return -EINVAL;
	ext->fe = gate_ps_parent(ext->cpu);
	if (!gate_ps_dispext(ext->fe, "fe", &ext->index))
		return -EINVAL;
	ext->sys = gate_ps_parent(ext->fe);
	if (!gate_ps_dispext(ext->sys, "sys", &ext->index))
		return -EINVAL;
	for (i = 0; i < 2; i++) {
		domain = gate_ps_parent(i ? ext->mbox : ext->dart);
		of_node_put(domain);
		if (domain != ext->cpu)
			return -EINVAL;
	}

	/* The driver waits for this request's acknowledgement before RUN. */
	ext->entry = gate_pmp_dispext_entry(ext->index);
	if (!ext->entry || !gate_disabled(ext->entry) || !ext->entry->phandle ||
	    !of_device_is_compatible(ext->entry, "apple,t6000-pmp-v2-report-entry") ||
	    of_property_read_u32(ext->entry, "reg", &id) ||
	    !of_property_read_bool(ext->entry, "apple,always-on") ||
	    of_property_read_bool(ext->entry, "apple,no-ack"))
		return -ENOENT;
	return 0;
}

/* Kept for good once applied: the live tree refers to it. */
static struct of_changeset *__init gate_cs_alloc(void)
{
	struct of_changeset *cs = kzalloc_obj(*cs);

	if (cs)
		of_changeset_init(cs);
	return cs;
}

static void __init gate_cs_free(struct of_changeset *cs)
{
	of_changeset_destroy(cs);
	kfree(cs);
}

/* Sets "status" of every Type-C route of @dcp that does not already have it. */
static int __init gate_ext_routes(struct of_changeset *cs, struct device_node *dcp,
				  const char *status)
{
	struct device_node *routes, *route;
	const char *now;
	int ret = 0;

	routes = of_get_child_by_name(dcp, "typec-routes");
	for_each_child_of_node(routes, route) {
		if (!of_property_read_string(route, "status", &now) && !strcmp(now, status))
			continue;
		ret = of_changeset_update_prop_string(cs, route, "status", status);
		if (ret) {
			of_node_put(route);
			break;
		}
	}
	of_node_put(routes);
	return ret;
}

/* The display subsystem's iommus with <@dart 0> appended. */
static int __init gate_display_add_dart(struct of_changeset *cs, struct device_node *dart)
{
	struct device_node *display = gate_find_one(gate_soc->display_compat);
	struct of_phandle_args args;
	struct property *old, *prop;
	__be32 *value;
	int i, n, ret = -EINVAL;

	if (!display)
		return -ENODEV;
	old = of_find_property(display, "iommus", NULL);
	n = of_count_phandle_with_args(display, "iommus", "#iommu-cells");
	if (!old || n < 1 || old->length != n * 2 * sizeof(__be32))
		goto out;
	for (i = 0; i < n; i++) {
		if (of_parse_phandle_with_args(display, "iommus", "#iommu-cells", i, &args))
			goto out;
		of_node_put(args.np);
		if (args.np == dart || args.args_count != 1)
			goto out;
	}
	ret = -ENOMEM;
	prop = kzalloc_obj(*prop);
	value = kmalloc(old->length + 2 * sizeof(*value), GFP_KERNEL);
	if (!prop || !value) {
		kfree(prop);
		kfree(value);
		goto out;
	}
	memcpy(value, old->value, old->length);
	value[old->length / sizeof(*value)] = cpu_to_be32(dart->phandle);
	value[old->length / sizeof(*value) + 1] = cpu_to_be32(0);
	prop->name = "iommus";
	prop->length = old->length + 2 * sizeof(*value);
	prop->value = value;
	ret = of_changeset_update_property(cs, display, prop);
out:
	of_node_put(display);
	return ret;
}

/*
 * The processor's PIODMA stream on its display DART, as the internal DCP's
 * piodma child: the driver maps firmware buffers there on request.
 */
static int __init gate_ext_piodma(struct of_changeset *cs, struct gate_ext *ext)
{
	u32 iommus[2] = { ext->disp_dart->phandle, 4 };
	struct device_node *piodma;
	int ret;

	piodma = of_get_child_by_name(ext->dcp, "piodma");
	if (piodma) {
		ret = of_changeset_update_prop_string(cs, piodma, "status", "okay");
		if (!ret) {
			struct property *prop = kzalloc_obj(*prop);
			__be32 *value = kmalloc(sizeof(iommus), GFP_KERNEL);

			if (!prop || !value) {
				kfree(prop);
				kfree(value);
				ret = -ENOMEM;
			} else {
				value[0] = cpu_to_be32(iommus[0]);
				value[1] = cpu_to_be32(iommus[1]);
				prop->name = "iommus";
				prop->length = sizeof(iommus);
				prop->value = value;
				ret = of_changeset_update_property(cs, piodma, prop);
			}
		}
		if (!ret)
			ret = gate_set_u32(cs, piodma, "apple,t6030-dispext-handoff", 1);
		of_node_put(piodma);
		return ret;
	}
	piodma = of_changeset_create_node(cs, ext->dcp, "piodma");
	if (!piodma)
		return -ENOMEM;
	/* The new node is kept for good, like the changeset that attaches it. */
	ret = of_changeset_add_prop_u32_array(cs, piodma, "iommus", iommus, ARRAY_SIZE(iommus));
	if (!ret)
		ret = of_changeset_add_prop_u32(cs, piodma, "apple,t6030-dispext-handoff", 1);
	return ret;
}

/* Enables @ext's processor, in the native or the manual configuration. */
static int __init gate_ext_apply(struct gate_ext *ext, bool native)
{
	struct of_changeset *cs = gate_cs_alloc();
	int ret;

	if (!cs)
		return -ENOMEM;
	/* PMGR applies the floor at probe, before the PMP can manage this CPU. */
	ret = gate_set_u32(cs, ext->cpu, "apple,min-state", PMGR_PS_ACTIVE);
	/* The driver must wait for this request's acknowledgement before RUN. */
	if (!ret)
		ret = gate_set_u32(cs, ext->dcp, "apple,pmp-report", ext->entry->phandle);
	/* No external display is attached in system sleep: the PMP may drop it. */
	if (!ret)
		ret = gate_add_bool(cs, ext->entry, "apple,release-for-sleep");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->entry, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->dart, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->mbox, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->dcp, "status", "okay");
	/* A processor's routes are usable exactly when the processor is. */
	if (!ret)
		ret = gate_ext_routes(cs, ext->dcp, "okay");
	if (!ret && ext->scanout_ok)
		ret = of_changeset_update_prop_string(cs, ext->disp_dart, "status", "okay");
	if (!ret && native) {
		ret = gate_ext_piodma(cs, ext);
		if (!ret)
			ret = gate_display_add_dart(cs, ext->disp_dart);
	} else if (!ret && ext->scanout_ok) {
		ret = gate_set_u32(cs, ext->scanout, "apple,t6030-scanout-verified", 1);
		if (!ret)
			ret = of_changeset_update_prop_string(cs, ext->scanout, "status", "okay");
	}
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret)
		gate_cs_free(cs);
	return ret;
}

/* Leaves @dcp disabled, and its routes with it. */
static void __init gate_ext_refuse(struct device_node *dcp, const char *why)
{
	struct of_changeset *cs;
	int ret = -ENOMEM;

	cs = gate_cs_alloc();
	if (cs) {
		ret = gate_ext_routes(cs, dcp, "disabled");
		if (!ret)
			ret = of_changeset_apply(cs);
		if (ret)
			gate_cs_free(cs);
	}
	gate_info("%pOF left disabled: %s%s\n", dcp, why,
		  ret ? "; its Type-C routes could not be disabled" : "");
}

/* The firmware the display driver runs external processors with. */
static bool __init gate_ext_firmware_ok(struct device_node *np)
{
	u32 ver[3];

	return of_property_count_u32_elems(np, "apple,firmware-compat") == 3 &&
	       !of_property_read_u32_array(np, "apple,firmware-compat", ver, 3) &&
	       ver[0] == 14 && ver[1] == 7 && ver[2] == 0;
}

static void __init gate_dcpext_one(struct device_node *np, const char *refuse)
{
	struct gate_ext ext = { .dcp = of_node_get(np) };
	bool native = !gate_dcpext_manual;
	int ret;

	if (!gate_disabled(np)) {
		gate_warn("%pOF is not disabled; external display gate skipped\n", np);
		goto put;
	}
	if (refuse) {
		gate_ext_refuse(np, refuse);
		goto put;
	}
	ret = gate_ext_resolve(&ext);
	if (ret == -ENODEV) {
		gate_ext_refuse(np, "the boot loader did not hand it over");
		goto put;
	}
	if (ret) {
		gate_ext_refuse(np, "power, memory or PMP prerequisites are not described");
		goto put;
	}
	/*
	 * The display driver refuses any other firmware at probe. Its Type-C
	 * routes would then never register, and every port that lists them
	 * would wait for them.
	 */
	if (!gate_ext_firmware_ok(np)) {
		gate_ext_refuse(np, "its firmware-compat is not 14.7.0");
		goto put;
	}
	if (gate_scanout_requested) {
		ret = gate_dispext_check(&ext);
		if (ret && ret != -ENODEV)
			gate_warn("%pOF: display DART handoff failed validation (%d)\n", np, ret);
	}
	if (native && !ext.scanout_ok) {
		gate_ext_refuse(np, "no usable display DART handoff");
		goto put;
	}
	ret = gate_ext_apply(&ext, native);
	if (ret) {
		gate_warn("%pOF: changeset failed: %d\n", np, ret);
		gate_ext_refuse(np, "the gate could not enable it");
		goto put;
	}
	if (native)
		gate_info("enabled dcpext%u (%pOF) with PMP DISPEXT%u request, CPU power floor, display DART %pOF and its Type-C routes; started by the display driver on first use\n",
			  ext.index, np, ext.index, ext.disp_dart);
	else
		gate_info("enabled dcpext%u (%pOF) for manual start%s\n", ext.index, np,
			  ext.scanout_ok ? " with the verified scanout DART" : "");
put:
	gate_ext_put(&ext);
}

/* Every external processor; @refuse, if set, says why none is enabled. */
static void __init gate_dcpext(const char *refuse)
{
	struct device_node *np;

	if (!refuse && !gate_dcpext_requested)
		refuse = "disabled on the command line";
	for_each_compatible_node(np, NULL, "apple,t6030-dcpext")
		gate_dcpext_one(np, refuse);
}

/* A one-cell property of @np as text, or "-" if there is none. */
static const char *__init gate_u32_text(const struct device_node *np, const char *name,
					char *buf, size_t len)
{
	u32 val;

	if (!np || of_property_count_u32_elems(np, name) != 1 ||
	    of_property_read_u32(np, name, &val))
		return "-";
	snprintf(buf, len, "%#x", val);
	return buf;
}

/* A firmware version property (3 to 5 cells) of @np as text, or "-". */
static const char *__init gate_version_text(const struct device_node *np, const char *name,
					    char *buf, size_t len)
{
	u32 ver[5];
	int i, n, pos;

	n = np ? of_property_read_variable_u32_array(np, name, ver, 3, ARRAY_SIZE(ver)) : -EINVAL;
	if (n < 3)
		return "-";
	pos = scnprintf(buf, len, "%u", ver[0]);
	for (i = 1; i < n; i++)
		pos += scnprintf(buf + pos, len - pos, ".%u", ver[i]);
	return buf;
}

/*
 * What the boot loader handed over, as found before the gate changes
 * anything: the markers, the DCP firmware identity and memory, this Mac's
 * PMP values and the DCP OS log reservations. Logged on every boot, also
 * when the native display is not requested.
 */
static void __init gate_describe(void)
{
	struct device_node *dcp, *display, *piodma = NULL, *values, *resv, *node;
	char m_dcp[12], m_disp[12], m_pio[12], compat[48], version[48], bdid[12], dvid[12];
	const char *uuid = "-", *pmp_uuid = "-";
	int i, regions = 0, oslog = 0, tunables = 0, reserved = 0;
	struct property *prop;
	struct resource res;

	dcp = gate_find_one(gate_soc->dcp_compat);
	display = gate_find_one(gate_soc->display_compat);
	if (dcp) {
		piodma = of_get_child_by_name(dcp, "piodma");
		of_property_read_string(dcp, "apple,firmware-uuid", &uuid);
		for (i = 0; (node = of_parse_phandle(dcp, "memory-region", i)); i++) {
			regions++;
			if (of_property_read_bool(node, "apple,dcp-os-log"))
				oslog++;
			of_node_put(node);
		}
	}
	gate_info("handoff: %s dcp %s display %s piodma %s; dcp firmware-compat %s version %s uuid %s; %d memory regions, %d OS log\n",
		  gate_soc->marker,
		  gate_u32_text(dcp, gate_soc->marker, m_dcp, sizeof(m_dcp)),
		  gate_u32_text(display, gate_soc->marker, m_disp, sizeof(m_disp)),
		  gate_u32_text(piodma, gate_soc->marker, m_pio, sizeof(m_pio)),
		  gate_version_text(dcp, "apple,firmware-compat", compat, sizeof(compat)),
		  gate_version_text(dcp, "apple,firmware-version", version, sizeof(version)),
		  uuid, regions, oslog);

	values = of_find_node_by_path(gate_soc->pmp_values);
	if (values) {
		of_property_read_string(values, "apple,tunable-uuid", &pmp_uuid);
		for_each_property_of_node(values, prop)
			if (strstarts(prop->name, "apple,tunable-"))
				tunables++;
		gate_info("handoff: PMP values %pOF: board-id %s dram-vendor-id %s tunable-uuid %s, %d tunables (image pinned: %s)\n",
			  values, gate_u32_text(values, "apple,board-id", bdid, sizeof(bdid)),
			  gate_u32_text(values, "apple,dram-vendor-id", dvid, sizeof(dvid)),
			  pmp_uuid, tunables, gate_soc->pmp_uuid);
	} else {
		gate_info("handoff: no PMP values (%s)\n", gate_soc->pmp_values);
	}

	resv = of_find_node_by_path("/reserved-memory");
	for_each_child_of_node(resv, node) {
		if (!of_node_name_prefix(node, "dcp-oslog"))
			continue;
		reserved++;
		if (!of_address_to_resource(node, 0, &res))
			gate_info("handoff: %pOFn %pR%s\n", node, &res,
				  of_property_read_bool(node, "no-map") ? " no-map" : "");
	}
	gate_info("handoff: %d dcp-oslog reservations\n", reserved);

	if (gate_soc->fw_uuids) {
		const char *image;

		i = 0;
		of_property_for_each_string(of_chosen, gate_soc->fw_uuids, prop, image) {
			gate_info("handoff: firmware image %s\n", image);
			i++;
		}
		if (!i)
			gate_info("handoff: no /chosen/%s\n", gate_soc->fw_uuids);
	}

	of_node_put(resv);
	of_node_put(values);
	of_node_put(piodma);
	of_node_put(display);
	of_node_put(dcp);
}

/*
 * The T8122 (M3) external display processor, dcpext. Unlike T6030's, the
 * boot loader does not hand it over: the display driver starts its firmware
 * itself, as on M1/M2 (DCP_FIRMWARE_V_14_7_TEMPLATE). The boot loader only
 * reserves that firmware (memory-region and iommu-addresses), sets its
 * firmware version and marks the node apple,t8122-dcpext-firmware-ready =
 * <1>. The node stays disabled in the device tree, so a kernel that cannot
 * drive it never sees it, and neither does a USB-C port controller waiting
 * for its Type-C display route. So do its DARTs, its mailbox and the display
 * crossbars of its routes, and the ATC power states are left as they are:
 * without the marker, nothing of the dcpext's probes or stays powered.
 *
 * Once the internal display was handed over, this enables, in one
 * changeset:
 *  - the dcpext node and its Type-C routes (a processor's routes are usable
 *    exactly when the processor is);
 *  - its DART, its display DART and its mailbox;
 *  - the display crossbars its routes switch (mux-controls), and
 *    apple,always-on on the common power state of each crossbar's ATC, as
 *    the M1/M2 laptops have for DP alt mode across suspend;
 *  - <&dispextN_dart 0> in the display subsystem's iommus, so that every
 *    framebuffer is mapped for the external pipe too, as on M1/M2 (the
 *    display DART is the one the dcpext's piodma child names);
 *  - an active floor on the dcpext's CPU power state, as on T6030, so that
 *    the running PMP cannot power the processor down under its firmware;
 *  - its DISPEXT request, the report entry the dcpext names in
 *    apple,pmp-report, made always on, as on T6030: once the PMP runs it
 *    owns the SoC power and performance states and keeps a block up only if
 *    that block's request bit was set when it started. The display driver
 *    waits for its acknowledgment before it runs the dcpext CPU.
 * The DARTs, the mailbox and the crossbars must be described and still
 * disabled. Without the marker, with other firmware, or if anything does not
 * check out, all of them stay disabled and the ATC power states are left
 * alone. apple_t8122_display.dcpext=0 keeps them disabled too.
 */
static bool gate_t8122_dcpext_requested __initdata = true;

static int __init gate_t8122_dcpext_setup(char *arg)
{
	return kstrtobool(arg, &gate_t8122_dcpext_requested);
}
early_param("apple_t8122_display.dcpext", gate_t8122_dcpext_setup);

/* The most display crossbars a T8122 dcpext's routes switch: ATC0's and ATC1's. */
#define GATE_T8122_XBARS	2

/*
 * The ATC common power state above display crossbar @xbar: the crossbar sits
 * in atcN_usb, one of whose parents is atcN_common.
 */
static struct device_node *__init gate_t8122_atc_common(struct device_node *xbar)
{
	struct device_node *usb, *found = NULL;
	struct of_phandle_args args;
	const char *label;
	char want[32];
	size_t len;
	int i, n;

	usb = gate_ps_parent(xbar);
	if (!usb || of_property_read_string(usb, "label", &label))
		goto out;
	len = strlen(label);
	if (!strstarts(label, "atc") || len < 8 || strcmp(label + len - 4, "_usb"))
		goto out;
	snprintf(want, sizeof(want), "%.*s_common", (int)(len - 4), label);
	n = of_count_phandle_with_args(usb, "power-domains", "#power-domain-cells");
	for (i = 0; i < n && !found; i++) {
		if (of_parse_phandle_with_args(usb, "power-domains", "#power-domain-cells",
					       i, &args))
			break;
		if (!args.args_count && gate_ps_is(args.np, want))
			found = args.np;
		else
			of_node_put(args.np);
	}
out:
	of_node_put(usb);
	return found;
}

/* Is @np among the first @n entries of @nodes? */
static bool __init gate_node_in(struct device_node *const *nodes, int n,
				const struct device_node *np)
{
	while (n--)
		if (nodes[n] == np)
			return true;
	return false;
}

static void __init gate_t8122_dcpext_one(struct device_node *np)
{
	struct device_node *dart = NULL, *mbox = NULL, *piodma = NULL;
	struct device_node *disp_dart = NULL, *ps = NULL, *entry = NULL, *link;
	struct device_node *xbar[GATE_T8122_XBARS] = {};
	struct device_node *common[GATE_T8122_XBARS] = {};
	struct of_phandle_args args;
	struct of_changeset *cs;
	const char *label;
	int ret, i, nxbar = 0;
	u32 ready;

	if (!gate_disabled(np)) {
		gate_warn("%pOF is not disabled; external display gate skipped\n", np);
		return;
	}
	if (of_property_read_u32(np, "apple,t8122-dcpext-firmware-ready", &ready) ||
	    ready != 1) {
		gate_ext_refuse(np, "the boot loader did not reserve its firmware");
		return;
	}
	if (!gate_ext_firmware_ok(np)) {
		gate_ext_refuse(np, "its firmware-compat is not 14.7.0");
		return;
	}
	if (!of_property_present(np, "memory-region")) {
		gate_ext_refuse(np, "it has no firmware memory");
		goto put;
	}
	dart = gate_target(np, "iommus", "#iommu-cells", 1, 5, "apple,t8110-dart");
	mbox = gate_target(np, "mboxes", "#mbox-cells", 0, 0, "apple,asc-mailbox-v4");
	piodma = of_get_child_by_name(np, "piodma");
	if (piodma)
		disp_dart = gate_target(piodma, "iommus", "#iommu-cells", 1, 4,
					"apple,t8110-dart");
	if (!dart || !mbox || !disp_dart || dart == disp_dart || !gate_disabled(dart) ||
	    !gate_disabled(mbox) || !gate_disabled(disp_dart)) {
		gate_ext_refuse(np, "its DARTs or mailbox are not described or not disabled");
		goto put;
	}
	ret = of_count_phandle_with_args(np, "mux-controls", "#mux-control-cells");
	if (ret != -ENOENT && (ret < 0 || ret > GATE_T8122_XBARS)) {
		gate_ext_refuse(np, "its display crossbars are not described");
		goto put;
	}
	for (i = 0; i < ret; i++, nxbar++) {
		if (of_parse_phandle_with_args(np, "mux-controls", "#mux-control-cells",
					       i, &args))
			break;
		xbar[i] = args.np;
		if (args.args_count != 1 || args.args[0] ||
		    !of_device_is_compatible(xbar[i], "apple,t6020-display-crossbar") ||
		    !gate_disabled(xbar[i]))
			break;
		common[i] = gate_t8122_atc_common(xbar[i]);
		if (!common[i])
			break;
	}
	if (i < ret) {
		gate_ext_refuse(np,
				"a display crossbar or its ATC power state is not described, or the crossbar is not disabled");
		goto put;
	}
	ps = of_parse_phandle(np, "power-domains", 0);
	if (!ps || !of_device_is_compatible(ps, gate_soc->pwrstate_compat) ||
	    of_property_read_string(ps, "label", &label) || strcmp(label, "dispext_cpu")) {
		gate_ext_refuse(np, "its power state is not dispext_cpu");
		goto put;
	}

	/*
	 * The PMP DISPEXT request: the first external mode set does not
	 * complete without it, so there is no dcpext without it, as on T6030.
	 * It is the device tree's own entry, which the dcpext names in
	 * apple,pmp-report, and nothing requests it yet.
	 */
	entry = gate_reuse.report ?
		gate_report_entry(gate_reuse.report, gate_soc->dispext_entry) : NULL;
	link = of_parse_phandle(np, "apple,pmp-report", 0);
	of_node_put(link);
	if (!entry || !entry->phandle || link != entry ||
	    of_property_read_bool(entry, "apple,always-on") ||
	    of_property_read_bool(entry, "apple,no-ack")) {
		gate_ext_refuse(np, "its PMP DISPEXT request is not described or not usable");
		goto put;
	}

	cs = gate_cs_alloc();
	if (!cs) {
		gate_ext_refuse(np, "out of memory");
		goto put;
	}
	ret = gate_set_u32(cs, ps, "apple,min-state", PMGR_PS_ACTIVE);
	/*
	 * The driver waits for this request's acknowledgment before RUN. It
	 * is seeded with the display's, by soc-device record, and a missing
	 * acknowledgment must not hold up the panel.
	 */
	if (!ret)
		ret = gate_add_bool(cs, entry, "apple,always-on");
	if (!ret)
		ret = gate_add_bool(cs, entry, "apple,soc-device-record");
	if (!ret)
		ret = gate_add_bool(cs, entry, "apple,defer-startup-ack");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, dart, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, disp_dart, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, mbox, "status", "okay");
	/* DP alt mode keeps the ATC's common power state on, as on M1/M2. */
	for (i = 0; i < nxbar && !ret; i++) {
		if (!gate_node_in(xbar, i, xbar[i]))
			ret = of_changeset_update_prop_string(cs, xbar[i], "status", "okay");
		if (!ret && !gate_node_in(common, i, common[i]) &&
		    !of_property_read_bool(common[i], "apple,always-on"))
			ret = of_changeset_add_prop_bool(cs, common[i], "apple,always-on");
	}
	if (!ret)
		ret = of_changeset_update_prop_string(cs, np, "status", "okay");
	if (!ret)
		ret = gate_ext_routes(cs, np, "okay");
	if (!ret)
		ret = gate_display_add_dart(cs, disp_dart);
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret) {
		gate_cs_free(cs);
		gate_warn("%pOF: changeset failed: %d\n", np, ret);
		gate_ext_refuse(np, "the gate could not enable it");
		goto put;
	}
	gate_info("enabled %pOF with its Type-C routes, DARTs, mailbox, %d display crossbar(s) with ATC common power kept on, display DART %pOF in the display subsystem, CPU power floor and PMP DISPEXT request; started by the display driver\n",
		  np, nxbar, disp_dart);
put:
	for (i = 0; i < GATE_T8122_XBARS; i++) {
		of_node_put(common[i]);
		of_node_put(xbar[i]);
	}
	of_node_put(entry);
	of_node_put(ps);
	of_node_put(disp_dart);
	of_node_put(piodma);
	of_node_put(mbox);
	of_node_put(dart);
}

static void __init gate_t8122_dcpext(bool panel)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "apple,t8122-dcpext") {
		if (!panel)
			gate_ext_refuse(np, "the internal display was not handed over");
		else if (!gate_t8122_dcpext_requested)
			gate_ext_refuse(np, "disabled on the command line");
		/*
		 * Enabled routes make the Type-C ports wait for the display
		 * driver's mode switches: without a display driver that would
		 * take USB from both ports of a J613.
		 */
		else if (!IS_ENABLED(CONFIG_DRM_APPLE))
			gate_ext_refuse(np, "the display driver is not built");
		else if (IS_ENABLED(CONFIG_VIDEO) && video_firmware_drivers_only())
			gate_ext_refuse(np, "nomodeset is on the command line");
		else
			gate_t8122_dcpext_one(np);
	}
}

/*
 * T6031 external displays: the J516C HDMI port
 *
 * The boot loader does not describe the T6031 external display processors
 * (dcpextN) at all. iBoot preloads the DCP firmware on every one of them and
 * leaves its segments in memory; the boot loader exports its ADT as the
 * phram region "adt", reserves each processor's OS log and names their
 * firmware in /chosen/asahi,t6031-fw-uuids. With apple_t6031_display.ext=1
 * on the kernel command line, and only once this same pass has handed the
 * internal display and the PMP over, the gate describes dcpext0, which
 * drives the HDMI port, from that ADT. It reads where dcpext0's segments are
 * and ATC3's common tunables (its calibration), checks them and the device
 * tree, and in one changeset:
 *  - reserves the DATA and heap segments at their DART addresses, adds the
 *    processor to the TEXT reservation it shares with the panel and names
 *    its OS log reservation;
 *  - adds dcpext0, its two DARTs, its mailbox and the clock its firmware is
 *    told about, and the HDMI port as the T6030 one is described
 *    (t603x-j514s-j516s-hdmi.dtsi): ATC3 as a DisplayPort-only PHY with the
 *    common tunables the boot loader gives T6030's ATC3, from the ADT's
 *    atc-phy3, the ATC3 display crossbar, the converter's GPIOs and the
 *    graph link through which the processor puts the PHY in DisplayPort
 *    mode;
 *  - hands the processor to the display driver's native external path as
 *    gate_ext_apply() does on T6030: the gate's memory and display DART
 *    markers, a piodma child on the display DART's PIODMA stream, an active
 *    floor on its CPU power state, and its PMP DISPEXT0 request, whose
 *    acknowledgment the display driver waits for before it runs dcpext0;
 *  - puts that request before dispext0's power instead of seeding it as
 *    T6030 and T8122 do: the report entry is taken out of dispext0_fe, sets
 *    its request only once the PMP has started (apple,request-after-start)
 *    and becomes the second parent of dispext0_sys. The PMP thus starts as
 *    it does without this phase, with the panel's requests alone and
 *    dispext0 not powered by Linux; the request is set once it is ready,
 *    and dispext0_sys, _cpu and _fe, and the DARTs, the mailbox and dcpext0
 *    in them, probe and power up only once the PMP has acknowledged it;
 *  - gives it a display subsystem of its own, bound by apple-drm-ext, so
 *    that the panel's DRM device neither waits for dcpext0 nor maps its
 *    framebuffers through dcpext0's display DART.
 * The T6031 power states nest the other way round from T6030's: dcpext0, its
 * DARTs and its mailbox are in dispext0_fe, under dispext0_cpu under
 * dispext0_sys, as the panel's DCP is in disp_fe.
 *
 * Nothing of it is applied unless every check passes, and with
 * apple_t6031_display.ext=0 (the default) none of this runs. The device tree
 * must not describe any of these nodes already. Only DCPEXT0 is described:
 * the USB-C display routes of dcpext1-3 are not.
 */

/* apple_t6031_display.ext: the external processors to describe, bit N for dcpextN. */
static unsigned int gate_t6031_ext __initdata;

/*
 * apple_t6031_display.ext_clock: the display clock rate, in Hz, that
 * dcpext0's firmware is told (D408). No ADT gives one for the external
 * processors; the default is the T6030 one, unconfirmed on T6031.
 */
#define GATE_T6031_EXT_CLOCK		935000000
static unsigned int gate_t6031_ext_clock __initdata = GATE_T6031_EXT_CLOCK;

static int __init gate_setup_t6031_ext(char *arg)
{
	return arg ? kstrtouint(arg, 0, &gate_t6031_ext) : -EINVAL;
}
early_param("apple_t6031_display.ext", gate_setup_t6031_ext);

static int __init gate_setup_t6031_ext_clock(char *arg)
{
	return arg ? kstrtouint(arg, 0, &gate_t6031_ext_clock) : -EINVAL;
}
early_param("apple_t6031_display.ext_clock", gate_setup_t6031_ext_clock);

/* The outputs described: DCPEXT0's HDMI port. */
#define GATE_T6031_EXT_OUTPUTS		BIT(0)

/* DCPEXT0's display block (ADT dispext0 reg[0]) and the windows in it. */
#define GATE_T6031_DISPEXT0		0x304000000ULL
#define GATE_T6031_EXT_DISP_DART	0x1304000	/* dart-dispext0 reg[0] */
#define GATE_T6031_EXT_DCP_DART		0x130c000	/* dart-dcpext0 reg[0] */
#define GATE_T6031_EXT_VBAR		0x2850000	/* dcpext0 reg[1] */
#define GATE_T6031_EXT_COPROC		0x2c00000	/* dcpext0 reg[0] */
#define GATE_T6031_EXT_MBOX		0x2c08000	/* coprocessor + 0x8000 */
/* Both DARTs share one interrupt; the mailbox has four, in ascending order. */
#define GATE_T6031_EXT_DART_IRQ		1047
#define GATE_T6031_EXT_MBOX_IRQ		1068
/* The AIC's interrupt cells: <AIC_IRQ die irq IRQ_TYPE_LEVEL_HIGH>. */
#define GATE_AIC_IRQ			0
#define GATE_IRQ_LEVEL_HIGH		4

/* DISPEXT0's PMP request: soc-device ID 18, request and acknowledge bit 17. */
#define GATE_T6031_DISPEXT0_REQ		17

/*
 * The DART window and addresses of the segments iBoot preloads for every
 * DCPEXT (iop-dcpextN-nub segment-ranges): the TEXT the panel's DCP runs
 * too, DATA, the OS log, which the firmware names by physical address, and
 * the heap, dcpext0's at the base of the heap window.
 */
#define GATE_T6031_FW_DVA_BASE		0x10000000000ULL	/* DART vm-base */
#define GATE_T6031_FW_DVA_SIZE		0x1000000000ULL		/* DART vm-size */
#define GATE_T6031_FW_TEXT_DVA		0x10040000000ULL
#define GATE_T6031_FW_DATA_DVA		0x10080000000ULL
#define GATE_T6031_FW_HEAP_DVA		0x100c0000000ULL

enum {
	GATE_SEG_TEXT,
	GATE_SEG_DATA,
	GATE_SEG_OSLOG,
	GATE_SEG_HEAP,
	GATE_SEG_NR,
};

static const u32 gate_t6031_seg_flags[GATE_SEG_NR] __initconst = { 0x1, 0x0, 0xa, 0x2 };

/*
 * The HDMI port: DCPEXT0 through ATC3, a fixed four-lane DisplayPort
 * transmitter, input 0 of its display crossbar and the DP to HDMI converter
 * (ADT atc-phy3, atc3-dpxbar, atc3-dpphy, atc3-hdmi and dp2hdmi-gpio0).
 */
#define GATE_T6031_ATC3			0x1303000000ULL
#define GATE_T6031_ATC3_SIZE		0x4c000
#define GATE_T6031_ATC3_XBAR		0x130304c000ULL
#define GATE_T6031_HDMI_DPTX_PHY	3	/* atc3-dpphy dp-switch-dfp-endpoint */
#define GATE_T6031_HDMI_XBAR_INPUT	0	/* dcpext0 dp-switch-ufp-endpoint */
#define GATE_T6031_HDMI_HPD		42	/* aop-gpio0, atc3-hdmi function-hdmi_hpd */
#define GATE_T6031_HDMI_PWREN		15	/* SMC gP0f, hdmi_pwr_en */
#define GATE_T6031_DP2HDMI_PWREN	26	/* SMC gP1a, dp2hdmi_pwr_en */
#define GATE_T6031_AOP_PINCTRL		"/soc/pinctrl@2a8824000"

/*
 * ATC3's common tunables ("apple,tunable-common-b"), built from the ADT's
 * atc-phy3 as the boot loader builds them for every T8122 generation PHY
 * (m1n1 atc_tunables_t8122), T6030's DisplayPort-only ATC3 included: these
 * blocks in this order, each entry at its block's offset in the core window,
 * with m1n1's block sizes; only CLKMON_CFG may be absent. The J516C's
 * atc-phy3 has all of them. The PHY driver resets the PHY when it probes and
 * writes these after every power-on, ahead of its own AUX, AUSPLL and lane
 * sequences: they are all the calibration the PHY gets.
 */
struct gate_t6031_atc_block {
	const char *name;
	u32 base, size;
	bool required;
};

static const struct gate_t6031_atc_block gate_t6031_atc3_common[] __initconst = {
	{ "tunable_ATC_FABRIC", 0x44000, 0x4000, true },
	{ "tunable_CIO3PLL_CORE", 0x2a00, 0x200, true },
	{ "tunable_CIO3PLL_TOP", 0x2800, 0x200, true },
	{ "tunable_ACIOPHY_LANE_USBC0", 0x5000, 0x1000, true },
	{ "tunable_ACIOPHY_PLL_TOP", 0x1000, 0x4000, true },
	{ "tunable_ACIOPHY_TOP", 0x0, 0x4000, true },
	{ "tunable_AUSCMN_DIG", 0x800, 0x200, true },
	{ "tunable_AUSPLL_CORE", 0x2200, 0x4000, true },
	{ "tunable_AUX_TOP", 0x16000, 0x4000, true },
	{ "tunable_AUSCMN_SHM", 0xa00, 0x200, true },
	{ "tunable_CLKMON_CFG", 0x2600, 0x100, false },
	{ "tunable_CIO_SHIM", 0x4000, 0x4000, true },
};

/* The J516C's atc-phy3 gives 93 of them. */
#define GATE_T6031_ATC3_COMMON_MAX	128

/* The nodes this phase adds; none of them may be in the device tree already. */
static const char *const gate_t6031_ext_paths[] __initconst = {
	"/soc/dcp@306c00000", "/soc/iommu@30530c000", "/soc/iommu@305304000",
	"/soc/mailbox@306c08000", "/soc/phy@1303000000", "/soc/mux@130304c000",
	"/clock-dispext0", "/dispext-subsystem",
};

/* The boot loader's ADT: a private copy of its phram "adt" export. */
struct gate_adt {
	const u8 *data;
	size_t size;
};

/* DCPEXT0's preloaded segments, as the ADT gives them, 16 KiB rounded. */
struct gate_t6031_fw {
	u64 phys[GATE_SEG_NR];
	u64 dva[GATE_SEG_NR];
	u64 size[GATE_SEG_NR];
	char uuid[37];
};

/* What the T6031 external phase found and checked, and its changeset. */
struct gate_t6031_ext {
	struct gate_t6031_fw fw;
	u32 atc3_common[3 * GATE_T6031_ATC3_COMMON_MAX];
	unsigned int nr_atc3_common;
	struct device_node *root, *soc, *rmem, *aic;
	struct device_node *fe, *cpu, *sys, *atc_common, *atc_usb;
	struct device_node *aop, *smc, *entry, *text, *oslog;
	u64 text_size;
	const char *uuid;
	u32 version[5];
	int nr_version;

	struct of_changeset *cs;
	phandle next_phandle;
	/* Nodes given a phandle in the changeset: attaching does not set it. */
	unsigned int nr_fix;
	struct device_node *fix[32];
	phandle fix_phandle[32];
	/* Nodes the changeset creates, held until the phase ends. */
	unsigned int nr_new;
	struct device_node *new[24];
};

/* The end of the ADT node at @off, or 0 if the node is malformed. */
static size_t __init gate_adt_node_end(const struct gate_adt *adt, size_t off,
				       unsigned int depth)
{
	u32 props, children, len, i;

	if (depth > 32 || off > adt->size || adt->size - off < 8)
		return 0;
	props = get_unaligned_le32(adt->data + off);
	children = get_unaligned_le32(adt->data + off + 4);
	if (props > 4096 || children > 1024)
		return 0;
	off += 8;
	for (i = 0; i < props; i++) {
		if (adt->size - off < 36)
			return 0;
		len = get_unaligned_le32(adt->data + off + 32) & 0x7fffffff;
		off += 36;
		if (len > adt->size - off)
			return 0;
		off += ALIGN(len, 4);
		if (off > adt->size)
			return 0;
	}
	for (i = 0; i < children; i++) {
		off = gate_adt_node_end(adt, off, depth + 1);
		if (!off)
			return 0;
	}
	return off;
}

/* A property of @node, a node of the tree gate_adt_load() checked. */
static const u8 *__init gate_adt_prop(const struct gate_adt *adt, size_t node,
				      const char *name, u32 *len)
{
	u32 props = get_unaligned_le32(adt->data + node), i, l;
	size_t off = node + 8;

	for (i = 0; i < props; i++) {
		l = get_unaligned_le32(adt->data + off + 32) & 0x7fffffff;
		if (!strncmp((const char *)adt->data + off, name, 32)) {
			*len = l;
			return adt->data + off + 36;
		}
		off += 36 + ALIGN(l, 4);
	}
	return NULL;
}

/* The child of @node named @name, or 0. */
static size_t __init gate_adt_child(const struct gate_adt *adt, size_t node, const char *name)
{
	u32 props = get_unaligned_le32(adt->data + node);
	u32 children = get_unaligned_le32(adt->data + node + 4);
	size_t off = node + 8;
	const u8 *n;
	u32 i, len;

	for (i = 0; i < props; i++)
		off += 36 + ALIGN(get_unaligned_le32(adt->data + off + 32) & 0x7fffffff, 4);
	for (i = 0; i < children; i++) {
		n = gate_adt_prop(adt, off, "name", &len);
		if (n && len && strnlen((const char *)n, len) == strlen(name) &&
		    !memcmp(n, name, strlen(name)))
			return off;
		off = gate_adt_node_end(adt, off, 1);
		if (!off)
			return 0;
	}
	return 0;
}

/* Cell @idx of a u32 array property, or U32_MAX if there is none. */
static u32 __init gate_adt_u32(const struct gate_adt *adt, size_t node, const char *name,
			       unsigned int idx)
{
	const u8 *p;
	u32 len;

	p = gate_adt_prop(adt, node, name, &len);
	return p && len / 4 > idx ? get_unaligned_le32(p + 4 * idx) : U32_MAX;
}

/* Whether the string property @name of @node starts with the string @str. */
static bool __init gate_adt_is(const struct gate_adt *adt, size_t node, const char *name,
			       const char *str)
{
	const u8 *p;
	u32 len;

	p = gate_adt_prop(adt, node, name, &len);
	return p && len > strlen(str) && !memcmp(p, str, strlen(str) + 1);
}

/*
 * Entry @idx of the reg of an /arm-io child, as a CPU address: /arm-io has
 * two address and two size cells, and its ranges are {child, parent, size}.
 */
static int __init gate_adt_reg(const struct gate_adt *adt, size_t armio, size_t node,
			       unsigned int idx, u64 *addr, u64 *size)
{
	const u8 *reg, *ranges;
	u64 a, child, parent, span;
	u32 len, rlen, off;

	reg = gate_adt_prop(adt, node, "reg", &len);
	ranges = gate_adt_prop(adt, armio, "ranges", &rlen);
	if (!reg || !ranges || len / 16 <= idx)
		return -ENOENT;
	a = get_unaligned_le64(reg + 16 * idx);
	*size = get_unaligned_le64(reg + 16 * idx + 8);
	for (off = 0; off + 24 <= rlen; off += 24) {
		child = get_unaligned_le64(ranges + off);
		parent = get_unaligned_le64(ranges + off + 8);
		span = get_unaligned_le64(ranges + off + 16);
		if (a >= child && a - child < span) {
			*addr = parent + (a - child);
			return 0;
		}
	}
	return -ERANGE;
}

/* A copy of the boot loader's phram "adt" export, checked as a whole tree. */
static int __init gate_adt_load(struct gate_adt *adt)
{
	struct device_node *rmem, *np;
	struct resource res;
	const char *label;
	void *map;
	u8 *copy;
	int ret = -ENODEV;

	rmem = of_find_node_by_path("/reserved-memory");
	for_each_child_of_node(rmem, np) {
		if (of_device_is_compatible(np, "phram") &&
		    !of_property_read_string(np, "label", &label) && !strcmp(label, "adt") &&
		    !of_address_to_resource(np, 0, &res)) {
			of_node_put(np);
			ret = 0;
			break;
		}
	}
	of_node_put(rmem);
	if (ret)
		return ret;
	if (!resource_size(&res) || resource_size(&res) > SZ_2M)
		return -EINVAL;
	map = memremap(res.start, resource_size(&res), MEMREMAP_WB);
	if (!map)
		return -ENOMEM;
	copy = kvmalloc(resource_size(&res), GFP_KERNEL);
	if (copy)
		memcpy(copy, map, resource_size(&res));
	memunmap(map);
	if (!copy)
		return -ENOMEM;
	adt->data = copy;
	adt->size = resource_size(&res);
	if (!gate_adt_node_end(adt, 0, 0)) {
		kvfree(copy);
		adt->data = NULL;
		return -EINVAL;
	}
	return 0;
}

static int __init gate_t6031_ext_refuse(const char *why)
{
	gate_warn("external displays not enabled: %s\n", why);
	return -EINVAL;
}

/*
 * DCPEXT0 in the ADT: its display block, processor, DARTs and interrupts
 * where this phase puts them, and its four preloaded segments in the
 * expected form, at the DART addresses the panel's firmware uses.
 */
static int __init gate_t6031_ext_adt(const struct gate_adt *adt, size_t armio,
				     struct gate_t6031_fw *fw)
{
	const u64 base = GATE_T6031_DISPEXT0;
	size_t disp, ctl, nub, dcp_dart, disp_dart;
	u64 addr, size, phys, dva;
	unsigned int i, irqs = 0;
	const char *what;
	const u8 *p;
	u32 len, irq;

	disp = gate_adt_child(adt, armio, "dispext0");
	ctl = gate_adt_child(adt, armio, "dcpext0");
	nub = ctl ? gate_adt_child(adt, ctl, "iop-dcpext0-nub") : 0;
	dcp_dart = gate_adt_child(adt, armio, "dart-dcpext0");
	disp_dart = gate_adt_child(adt, armio, "dart-dispext0");
	what = "nodes";
	if (!disp || !nub || !dcp_dart || !disp_dart)
		goto bad;

	what = "addresses";
	if (gate_adt_reg(adt, armio, disp, 0, &addr, &size) || addr != base ||
	    gate_adt_reg(adt, armio, ctl, 0, &addr, &size) ||
	    addr != base + GATE_T6031_EXT_COPROC || size < SZ_16K ||
	    gate_adt_reg(adt, armio, ctl, 1, &addr, &size) || addr != base + GATE_T6031_EXT_VBAR ||
	    gate_adt_reg(adt, armio, dcp_dart, 0, &addr, &size) ||
	    addr != base + GATE_T6031_EXT_DCP_DART ||
	    gate_adt_reg(adt, armio, disp_dart, 0, &addr, &size) ||
	    addr != base + GATE_T6031_EXT_DISP_DART)
		goto bad;

	what = "interrupts";
	p = gate_adt_prop(adt, ctl, "interrupts", &len);
	for (i = 0; p && len == 16 && i < 4; i++) {
		irq = get_unaligned_le32(p + 4 * i);
		if (irq - GATE_T6031_EXT_MBOX_IRQ < 4)
			irqs |= BIT(irq - GATE_T6031_EXT_MBOX_IRQ);
	}
	if (irqs != 0xf ||
	    gate_adt_u32(adt, dcp_dart, "interrupts", 0) != GATE_T6031_EXT_DART_IRQ ||
	    gate_adt_u32(adt, disp_dart, "interrupts", 0) != GATE_T6031_EXT_DART_IRQ)
		goto bad;

	what = "firmware";
	p = gate_adt_prop(adt, nub, "uuid", &len);
	if (gate_adt_u32(adt, nub, "pre-loaded", 0) != 1 || !p || len < 37 ||
	    strnlen((const char *)p, len) != 36)
		goto bad;
	memcpy(fw->uuid, p, 37);

	what = "segments";
	p = gate_adt_prop(adt, nub, "segment-ranges", &len);
	if (!p || len != 32 * GATE_SEG_NR)
		goto bad;
	/* Each {u64 physical, u64 IOVA, u64 DART address, u32 size, u32 flags}. */
	for (i = 0; i < GATE_SEG_NR; i++, p += 32) {
		phys = get_unaligned_le64(p);
		dva = get_unaligned_le64(p + 16);
		size = ALIGN((u64)get_unaligned_le32(p + 24), SZ_16K);
		if (get_unaligned_le32(p + 28) != gate_t6031_seg_flags[i] || !size ||
		    size > SZ_64M || !IS_ALIGNED(phys, SZ_16K) || !IS_ALIGNED(dva, SZ_16K))
			goto bad;
		switch (i) {
		case GATE_SEG_TEXT:
			addr = GATE_T6031_FW_TEXT_DVA;
			break;
		case GATE_SEG_DATA:
			addr = GATE_T6031_FW_DATA_DVA;
			break;
		case GATE_SEG_OSLOG:
			addr = phys;
			break;
		default:
			addr = GATE_T6031_FW_HEAP_DVA;
			break;
		}
		if (dva != addr || (i != GATE_SEG_OSLOG &&
				    (dva < GATE_T6031_FW_DVA_BASE ||
				     dva - GATE_T6031_FW_DVA_BASE > GATE_T6031_FW_DVA_SIZE - size)))
			goto bad;
		fw->phys[i] = phys;
		fw->dva[i] = dva;
		fw->size[i] = size;
	}
	return 0;
bad:
	gate_warn("external displays not enabled: the ADT's DCPEXT0 %s are not the expected ones\n",
		  what);
	return -EINVAL;
}

/*
 * ATC3's common tunables from the ADT, as {core offset, mask, value}, with
 * the boot loader's checks (32-bit registers, aligned, inside their block)
 * and values inside their masks.
 */
static int __init gate_t6031_ext_atc3(const struct gate_adt *adt, size_t armio,
				      struct gate_t6031_ext *x)
{
	size_t phy = gate_adt_child(adt, armio, "atc-phy3");
	size_t xbar = gate_adt_child(adt, armio, "atc3-dpxbar");
	const struct gate_t6031_atc_block *blk;
	u32 len, word, off, mask, value, *out;
	unsigned int i, n = 0;
	u64 addr, size;
	const u8 *p;

	if (!phy || !xbar || !gate_adt_is(adt, phy, "compatible", "atc-phy,t8122") ||
	    gate_adt_reg(adt, armio, phy, 3, &addr, &size) || addr != GATE_T6031_ATC3 ||
	    gate_adt_reg(adt, armio, xbar, 0, &addr, &size) || addr != GATE_T6031_ATC3_XBAR)
		return gate_t6031_ext_refuse("the ADT's ATC3 nodes are not the expected ones");
	for (i = 0; i < ARRAY_SIZE(gate_t6031_atc3_common); i++) {
		blk = &gate_t6031_atc3_common[i];
		p = gate_adt_prop(adt, phy, blk->name, &len);
		if (!p && !blk->required)
			continue;
		if (!p || len % 12)
			goto bad;
		/* Each {u32 offset:24, size:8; u32 mask; u32 value}. */
		for (; len; len -= 12, p += 12) {
			word = get_unaligned_le32(p);
			off = word & 0xffffff;
			mask = get_unaligned_le32(p + 4);
			value = get_unaligned_le32(p + 8);
			if (word >> 24 != 32 || off % 4 || off + 4 > blk->size || (value & ~mask) ||
			    n == GATE_T6031_ATC3_COMMON_MAX)
				goto bad;
			out = &x->atc3_common[3 * n++];
			out[0] = blk->base + off;
			out[1] = mask;
			out[2] = value;
		}
	}
	if (!n)
		return gate_t6031_ext_refuse("the ADT's atc-phy3 has no common tunables");
	x->nr_atc3_common = n;
	return 0;
bad:
	gate_warn("external displays not enabled: the ADT's atc-phy3 %s is missing or not usable\n",
		  blk->name);
	return -EINVAL;
}

/*
 * The TEXT reservation dcpext0 shares with the panel: one of the panel DCP's
 * memory regions, holding that segment, mapped for the panel alone at the
 * segment's DART address.
 */
static struct device_node *__init gate_t6031_ext_text(struct device_node *panel,
						      const struct gate_t6031_fw *fw, u64 *size)
{
	struct resource res = {};
	struct device_node *np;
	const __be32 *map;
	int i, len;

	for (i = 0; (np = of_parse_phandle(panel, "memory-region", i)); i++) {
		if (of_device_is_compatible(np, "apple,asc-mem") &&
		    of_property_read_bool(np, "no-map") && !of_address_to_resource(np, 0, &res) &&
		    res.start == fw->phys[GATE_SEG_TEXT] &&
		    resource_size(&res) >= fw->size[GATE_SEG_TEXT])
			break;
		of_node_put(np);
	}
	/* One {owner, DART address, size}: the panel DCP's, two cells each under /soc. */
	map = np ? of_get_property(np, "iommu-addresses", &len) : NULL;
	if (!map || len != (int)(5 * sizeof(*map)) || be32_to_cpup(map) != panel->phandle ||
	    of_read_number(map + 1, 2) != fw->dva[GATE_SEG_TEXT] ||
	    of_read_number(map + 3, 2) != resource_size(&res)) {
		of_node_put(np);
		return NULL;
	}
	*size = resource_size(&res);
	return np;
}

/* The boot loader's reservation of dcpext0's OS log: its own, with no DART address. */
static struct device_node *__init gate_t6031_ext_oslog(struct device_node *rmem,
						       struct device_node *panel,
						       const struct gate_t6031_fw *fw)
{
	struct device_node *np, *found = NULL, *region;
	struct resource res;
	int i;

	for_each_child_of_node(rmem, np) {
		if (!of_property_read_bool(np, "apple,dcp-os-log") ||
		    !of_property_read_bool(np, "no-map") ||
		    of_property_present(np, "iommu-addresses") ||
		    of_address_to_resource(np, 0, &res) || res.start != fw->phys[GATE_SEG_OSLOG] ||
		    resource_size(&res) < fw->size[GATE_SEG_OSLOG])
			continue;
		if (found) {
			of_node_put(np);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(np);
	}
	for (i = 0; found && (region = of_parse_phandle(panel, "memory-region", i)); i++) {
		of_node_put(region);
		if (region == found) {
			of_node_put(found);
			return NULL;
		}
	}
	return found;
}

/* No memory Linux knows of, nor any reservation, overlaps [@start, @start + @size). */
static bool __init gate_t6031_ext_unclaimed(struct device_node *rmem, u64 start, u64 size)
{
	struct device_node *np;
	struct resource res;
	int i;

	if (region_intersects(start, size, IORESOURCE_MEM, IORES_DESC_NONE) != REGION_DISJOINT)
		return false;
	for_each_child_of_node(rmem, np) {
		for (i = 0; !of_address_to_resource(np, i, &res); i++) {
			if (res.start < start + size && start <= res.end) {
				of_node_put(np);
				return false;
			}
		}
	}
	return true;
}

/* Whether @parent is among the power domains of @np. */
static bool __init gate_ps_has_parent(struct device_node *np, struct device_node *parent)
{
	struct of_phandle_args args;
	bool found = false;
	int i, n;

	n = of_count_phandle_with_args(np, "power-domains", "#power-domain-cells");
	for (i = 0; i < n && !found; i++) {
		if (of_parse_phandle_with_args(np, "power-domains", "#power-domain-cells", i,
					       &args))
			break;
		found = args.np == parent && !args.args_count;
		of_node_put(args.np);
	}
	return found;
}

/* Everything dcpext0 needs from the device tree, checked without changing anything. */
static int __init gate_t6031_ext_resolve(struct gate_t6031_ext *x, struct device_node **np)
{
	const struct gate_t6031_fw *fw = &x->fw;
	struct device_node *node;
	const char *s, *panel_uuid = NULL;
	struct property *prop;
	unsigned int i;
	u32 cells, id;
	bool ok;

	x->root = of_find_node_by_path("/");
	x->soc = of_find_node_by_path("/soc");
	x->rmem = of_find_node_by_path("/reserved-memory");
	/* Every node and reservation added is written with two-cell addresses. */
	if (!x->root || !x->soc || !x->rmem || !of_chosen || np[GATE_DCP]->parent != x->soc ||
	    of_property_read_u32(x->soc, "#address-cells", &cells) || cells != 2 ||
	    of_property_read_u32(x->soc, "#size-cells", &cells) || cells != 2 ||
	    of_property_read_u32(x->rmem, "#address-cells", &cells) || cells != 2 ||
	    of_property_read_u32(x->rmem, "#size-cells", &cells) || cells != 2)
		return gate_t6031_ext_refuse("/soc or /reserved-memory not as expected");

	for (i = 0; i < ARRAY_SIZE(gate_t6031_ext_paths); i++) {
		node = of_find_node_by_path(gate_t6031_ext_paths[i]);
		of_node_put(node);
		if (node) {
			gate_warn("external displays not enabled: the device tree has %s already\n",
				  gate_t6031_ext_paths[i]);
			return -EEXIST;
		}
	}
	node = of_find_compatible_node(NULL, NULL, "apple,t6031-dcpext");
	if (!node)
		node = of_find_compatible_node(NULL, NULL, "apple,t6031-dispext-subsystem");
	of_node_put(node);
	if (node)
		return gate_t6031_ext_refuse("the device tree has a T6031 dcpext already");

	/* The interrupt controller of the panel's mailbox, for the nodes added. */
	x->aic = of_parse_phandle(np[GATE_DCP_MBOX], "interrupt-parent", 0);
	if (!x->aic || !x->aic->phandle ||
	    !of_property_read_bool(x->aic, "interrupt-controller") ||
	    of_property_read_u32(x->aic, "#interrupt-cells", &cells) || cells != 4)
		return gate_t6031_ext_refuse("no four-cell AIC parent for the panel's mailbox");

	/* The firmware: the panel's 14.7.0 image, as the boot loader and the ADT name it. */
	of_property_for_each_string(of_chosen, "asahi,t6031-fw-uuids", prop, s)
		if (strstarts(s, "dcpext0="))
			x->uuid = s + strlen("dcpext0=");
	of_property_read_string(np[GATE_DCP], "apple,firmware-uuid", &panel_uuid);
	x->nr_version = of_property_read_variable_u32_array(np[GATE_DCP], "apple,firmware-version",
							    x->version, 3,
							    ARRAY_SIZE(x->version));
	if (!x->uuid || !panel_uuid || strcmp(x->uuid, fw->uuid) || strcmp(x->uuid, panel_uuid) ||
	    !gate_ext_firmware_ok(np[GATE_DCP]) || x->nr_version < 3)
		return gate_t6031_ext_refuse("DCPEXT0's firmware is not the panel's 14.7.0 image");

	/*
	 * dispext0_fe under dispext0_cpu under dispext0_sys, as the panel's,
	 * and dispext0_sys under one other power state: the request becomes
	 * its second parent.
	 */
	x->fe = gate_ps_unique("dispext0_fe");
	x->cpu = gate_ps_unique("dispext0_cpu");
	x->sys = gate_ps_unique("dispext0_sys");
	node = x->fe ? gate_ps_parent(x->fe) : NULL;
	ok = node && node == x->cpu;
	of_node_put(node);
	node = x->cpu ? gate_ps_parent(x->cpu) : NULL;
	ok = ok && node && node == x->sys;
	of_node_put(node);
	node = x->sys ? gate_ps_parent(x->sys) : NULL;
	ok = ok && node;
	of_node_put(node);
	if (!ok)
		return gate_t6031_ext_refuse("dispext0_fe, _cpu and _sys do not nest as expected");
	x->atc_common = gate_ps_unique("atc3_common");
	x->atc_usb = gate_ps_unique("atc3_usb");
	if (!x->atc_common || !x->atc_usb || !gate_ps_has_parent(x->atc_usb, x->atc_common))
		return gate_t6031_ext_refuse("no atc3_usb power state under atc3_common");

	/* The converter's hotplug pin and supplies. */
	x->aop = of_find_node_by_path(GATE_T6031_AOP_PINCTRL);
	x->smc = gate_find_one("apple,smc-gpio");
	if (!x->aop || !x->smc || !of_device_is_compatible(x->aop, "apple,t6031-pinctrl") ||
	    !of_property_read_bool(x->aop, "gpio-controller") ||
	    !of_property_read_bool(x->aop, "interrupt-controller") ||
	    of_property_read_u32(x->aop, "#gpio-cells", &cells) || cells != 2 ||
	    of_property_read_u32(x->aop, "apple,npins", &cells) || cells <= GATE_T6031_HDMI_HPD ||
	    !of_property_read_bool(x->smc, "gpio-controller") ||
	    of_property_read_u32(x->smc, "#gpio-cells", &cells) || cells != 2)
		return gate_t6031_ext_refuse("no AOP pins or SMC GPIOs for the HDMI converter");

	/*
	 * Its PMP request, disabled until now, acknowledged by the PMP, not
	 * seeded, a power domain of its own and in dispext0_fe alone, which the
	 * changeset takes it out of.
	 */
	x->entry = gate_pmp_dispext_entry(0);
	node = x->entry ? gate_ps_parent(x->entry) : NULL;
	of_node_put(node);
	if (!x->entry || !gate_disabled(x->entry) || node != x->fe ||
	    !of_device_is_compatible(x->entry, "apple,t6000-pmp-v2-report-entry") ||
	    of_property_read_u32(x->entry, "reg", &id) || id != GATE_T6031_DISPEXT0_REQ ||
	    of_property_read_u32(x->entry, "#power-domain-cells", &cells) || cells ||
	    of_property_read_bool(x->entry, "apple,no-ack") ||
	    of_property_read_bool(x->entry, "apple,soc-device-record") ||
	    of_property_read_bool(x->entry, "apple,always-on") ||
	    of_property_read_bool(x->entry, "apple,request-after-start"))
		return gate_t6031_ext_refuse("DCPEXT0's PMP request is not as expected");

	/* TEXT shared with the panel, its own OS log, DATA and heap nobody's. */
	x->text = gate_t6031_ext_text(np[GATE_DCP], fw, &x->text_size);
	x->oslog = gate_t6031_ext_oslog(x->rmem, np[GATE_DCP], fw);
	if (!x->text || !x->oslog)
		return gate_t6031_ext_refuse("DCPEXT0's TEXT or OS log is not reserved");
	if (!gate_t6031_ext_unclaimed(x->rmem, fw->phys[GATE_SEG_DATA], fw->size[GATE_SEG_DATA]) ||
	    !gate_t6031_ext_unclaimed(x->rmem, fw->phys[GATE_SEG_HEAP], fw->size[GATE_SEG_HEAP]) ||
	    (fw->phys[GATE_SEG_DATA] < fw->phys[GATE_SEG_HEAP] + fw->size[GATE_SEG_HEAP] &&
	     fw->phys[GATE_SEG_HEAP] < fw->phys[GATE_SEG_DATA] + fw->size[GATE_SEG_DATA]))
		return gate_t6031_ext_refuse("DCPEXT0's DATA or heap overlaps known memory");
	return 0;
}

/* The first phandle no node has. */
static phandle __init gate_free_phandle(void)
{
	struct device_node *np;
	phandle last = 0;

	for_each_of_allnodes(np)
		last = max(last, np->phandle);
	return last + 1;
}

/* @np's phandle; a node without one gets the next free one, in the changeset. */
static int __init gate_x_phandle(struct gate_t6031_ext *x, struct device_node *np, phandle *ph)
{
	unsigned int i;

	if (np->phandle) {
		*ph = np->phandle;
		return 0;
	}
	for (i = 0; i < x->nr_fix; i++) {
		if (x->fix[i] == np) {
			*ph = x->fix_phandle[i];
			return 0;
		}
	}
	if (x->nr_fix == ARRAY_SIZE(x->fix))
		return -ENOSPC;
	*ph = x->next_phandle++;
	x->fix[x->nr_fix] = np;
	x->fix_phandle[x->nr_fix++] = *ph;
	return of_changeset_add_prop_u32(x->cs, np, "phandle", *ph);
}

/* A new node @name under @parent, with a phandle if @ph is set, or NULL. */
static struct device_node *__init gate_x_node(struct gate_t6031_ext *x,
					      struct device_node *parent, const char *name,
					      phandle *ph)
{
	struct device_node *np;

	if (!parent || x->nr_new == ARRAY_SIZE(x->new))
		return NULL;
	np = of_changeset_create_node(x->cs, parent, name);
	if (!np)
		return NULL;
	x->new[x->nr_new++] = np;
	if (ph && gate_x_phandle(x, np, ph))
		return NULL;
	return np;
}

/* @name set to @cells, or with @cells appended to its current value. */
static int __init gate_x_cells(struct gate_t6031_ext *x, struct device_node *np, const char *name,
			       const u32 *cells, unsigned int count, bool append)
{
	const struct property *old = of_find_property(np, name, NULL);
	size_t keep = append && old ? old->length : 0;
	struct property *prop;
	unsigned int i;
	__be32 *val;

	if (keep % sizeof(*val))
		return -EINVAL;
	/* Kept for good: the live tree refers to it once applied. */
	prop = kzalloc_obj(*prop);
	val = kmalloc(keep + count * sizeof(*val), GFP_KERNEL);
	if (!prop || !val) {
		kfree(prop);
		kfree(val);
		return -ENOMEM;
	}
	if (keep)
		memcpy(val, old->value, keep);
	for (i = 0; i < count; i++)
		val[keep / sizeof(*val) + i] = cpu_to_be32(cells[i]);
	prop->name = (char *)name;
	prop->value = val;
	prop->length = keep + count * sizeof(*val);
	return old ? of_changeset_update_property(x->cs, np, prop) :
		     of_changeset_add_property(x->cs, np, prop);
}

/* Removes the property @name, which @np must have. */
static int __init gate_x_remove(struct gate_t6031_ext *x, struct device_node *np,
				const char *name)
{
	struct property *prop = of_find_property(np, name, NULL);

	return prop ? of_changeset_remove_property(x->cs, np, prop) : -ENOENT;
}

/* {address, size} as two-cell numbers. */
static void __init gate_x_pair(u32 *cells, u64 addr, u64 size)
{
	cells[0] = upper_32_bits(addr);
	cells[1] = lower_32_bits(addr);
	cells[2] = upper_32_bits(size);
	cells[3] = lower_32_bits(size);
}

/* "reg" with one {@addr, @size}. */
static int __init gate_x_reg(struct gate_t6031_ext *x, struct device_node *np, u64 addr,
			     u64 size)
{
	u32 reg[4];

	gate_x_pair(reg, addr, size);
	return of_changeset_add_prop_u32_array(x->cs, np, "reg", reg, ARRAY_SIZE(reg));
}

/* @count AIC interrupts from @first, level high. */
static int __init gate_x_irqs(struct gate_t6031_ext *x, struct device_node *np, u32 first,
			      unsigned int count)
{
	u32 cells[4 * 4];
	unsigned int i;

	if (count > 4)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		cells[4 * i] = GATE_AIC_IRQ;
		cells[4 * i + 1] = 0;
		cells[4 * i + 2] = first + i;
		cells[4 * i + 3] = GATE_IRQ_LEVEL_HIGH;
	}
	return of_changeset_add_prop_u32(x->cs, np, "interrupt-parent", x->aic->phandle) ?:
	       of_changeset_add_prop_u32_array(x->cs, np, "interrupts", cells, 4 * count);
}

/* A DART of dcpext0's display block, at @addr, powered by @ps. */
static int __init gate_x_dart(struct gate_t6031_ext *x, struct device_node *np, u64 addr,
			      phandle ps)
{
	static const char *const compat[] = { "apple,t6031-dart", "apple,t8110-dart" };
	static const u32 dma_range[] = { 0x100, 0x0, 0x10, 0x0 };

	return of_changeset_add_prop_string_array(x->cs, np, "compatible", compat,
						  ARRAY_SIZE(compat)) ?:
	       gate_x_reg(x, np, addr, SZ_16K) ?:
	       gate_x_irqs(x, np, GATE_T6031_EXT_DART_IRQ, 1) ?:
	       of_changeset_add_prop_u32(x->cs, np, "#iommu-cells", 1) ?:
	       of_changeset_add_prop_u32_array(x->cs, np, "apple,dma-range", dma_range,
					       ARRAY_SIZE(dma_range)) ?:
	       of_changeset_add_prop_u32(x->cs, np, "power-domains", ps);
}

/* A new no-map reservation of a dcpext0 segment, at its DART address for @owner. */
static int __init gate_x_reserve(struct gate_t6031_ext *x, const char *what, unsigned int seg,
				 phandle owner, phandle *ph)
{
	struct device_node *np;
	char name[48];
	u32 map[5];

	snprintf(name, sizeof(name), "dcpext0-%s@%llx", what, x->fw.phys[seg]);
	np = gate_x_node(x, x->rmem, name, ph);
	if (!np)
		return -ENOMEM;
	map[0] = owner;
	gate_x_pair(map + 1, x->fw.dva[seg], x->fw.size[seg]);
	return of_changeset_add_prop_string(x->cs, np, "compatible", "apple,asc-mem") ?:
	       of_changeset_add_prop_bool(x->cs, np, "no-map") ?:
	       gate_x_reg(x, np, x->fw.phys[seg], x->fw.size[seg]) ?:
	       of_changeset_add_prop_u32_array(x->cs, np, "iommu-addresses", map,
					       ARRAY_SIZE(map));
}

/*
 * A one-port graph between dcpext0 (port 1, its DisplayPort output) and ATC3
 * (port 2, its DisplayPort input), as on T6030: the processor finds the
 * PHY's mode switch through it.
 */
static struct device_node *__init gate_x_port(struct gate_t6031_ext *x, struct device_node *np,
					      u32 port, phandle *ph)
{
	struct device_node *ports, *node;
	char name[16];

	ports = gate_x_node(x, np, "ports", NULL);
	snprintf(name, sizeof(name), "port@%u", port);
	node = gate_x_node(x, ports, name, NULL);
	if (!ports || !node ||
	    of_changeset_add_prop_u32(x->cs, ports, "#address-cells", 1) ||
	    of_changeset_add_prop_u32(x->cs, ports, "#size-cells", 0) ||
	    of_changeset_add_prop_u32(x->cs, node, "reg", port))
		return NULL;
	return gate_x_node(x, node, "endpoint", ph);
}

/* The changeset: see the description above gate_t6031_ext_paths. */
static int __init gate_t6031_ext_build(struct gate_t6031_ext *x, struct device_node **subsys)
{
	static const char *const mbox_compat[] = { "apple,t6031-asc-mailbox",
						   "apple,asc-mailbox-v4" };
	static const char *const mbox_irqs[] = { "send-empty", "send-not-empty",
						 "recv-empty", "recv-not-empty" };
	static const char *const phy_compat[] = { "apple,t6031-atcphy", "apple,t8122-atcphy" };
	static const char *const xbar_compat[] = { "apple,t6031-display-crossbar",
						   "apple,t6020-display-crossbar" };
	static const char *const dcp_compat[] = { "apple,t6031-dcpext", "apple,dcpext" };
	static const char *const regions[] = { "asc-firmware", "dcp_data", "os_log", "heap" };
	static const u32 fw_compat[] = { 14, 7, 0 };
	const u64 base = GATE_T6031_DISPEXT0;
	struct device_node *dcpext, *dcp_dart, *disp_dart, *mbox, *phy, *xbar, *clk;
	struct device_node *dcp_ep, *phy_ep, *piodma;
	phandle fe, atc_common, atc_usb, aop, smc, entry, region[GATE_SEG_NR];
	phandle ph_dcpext, ph_dcp_dart, ph_disp_dart, ph_mbox, ph_phy, ph_xbar, ph_clk;
	phandle ph_dcp_ep = 0, ph_phy_ep = 0;
	u32 cells[5];
	int ret;

	/* The nodes the new ones refer to. */
	ret = gate_x_phandle(x, x->fe, &fe) ?:
	      gate_x_phandle(x, x->atc_common, &atc_common) ?:
	      gate_x_phandle(x, x->atc_usb, &atc_usb) ?:
	      gate_x_phandle(x, x->aop, &aop) ?:
	      gate_x_phandle(x, x->smc, &smc) ?:
	      gate_x_phandle(x, x->entry, &entry) ?:
	      gate_x_phandle(x, x->text, &region[GATE_SEG_TEXT]) ?:
	      gate_x_phandle(x, x->oslog, &region[GATE_SEG_OSLOG]);
	if (ret)
		return ret;

	/* The new nodes, and the phandles they refer to each other by. */
	dcpext = gate_x_node(x, x->soc, "dcp@306c00000", &ph_dcpext);
	dcp_dart = gate_x_node(x, x->soc, "iommu@30530c000", &ph_dcp_dart);
	disp_dart = gate_x_node(x, x->soc, "iommu@305304000", &ph_disp_dart);
	mbox = gate_x_node(x, x->soc, "mailbox@306c08000", &ph_mbox);
	phy = gate_x_node(x, x->soc, "phy@1303000000", &ph_phy);
	xbar = gate_x_node(x, x->soc, "mux@130304c000", &ph_xbar);
	clk = gate_x_node(x, x->root, "clock-dispext0", &ph_clk);
	dcp_ep = dcpext ? gate_x_port(x, dcpext, 1, &ph_dcp_ep) : NULL;
	phy_ep = phy ? gate_x_port(x, phy, 2, &ph_phy_ep) : NULL;
	piodma = gate_x_node(x, dcpext, "piodma", NULL);
	*subsys = gate_x_node(x, x->root, "dispext-subsystem", NULL);
	if (!dcpext || !dcp_dart || !disp_dart || !mbox || !phy || !xbar || !clk ||
	    !dcp_ep || !phy_ep || !piodma || !*subsys)
		return -ENOMEM;

	/* DATA and heap, new; TEXT, the panel's, mapped for dcpext0 too. */
	cells[0] = ph_dcpext;
	gate_x_pair(cells + 1, GATE_T6031_FW_TEXT_DVA, x->text_size);
	ret = gate_x_reserve(x, "data", GATE_SEG_DATA, ph_dcpext, &region[GATE_SEG_DATA]) ?:
	      gate_x_reserve(x, "heap", GATE_SEG_HEAP, ph_dcpext, &region[GATE_SEG_HEAP]) ?:
	      gate_x_cells(x, x->text, "iommu-addresses", cells, 5, true);
	if (ret)
		return ret;

	/* The clock the firmware is told, the DARTs and the mailbox, all in dispext0_fe. */
	ret = of_changeset_add_prop_string(x->cs, clk, "compatible", "fixed-clock") ?:
	      of_changeset_add_prop_u32(x->cs, clk, "#clock-cells", 0) ?:
	      of_changeset_add_prop_u32(x->cs, clk, "clock-frequency", gate_t6031_ext_clock) ?:
	      of_changeset_add_prop_string(x->cs, clk, "clock-output-names", "clk_dispext0") ?:
	      gate_x_dart(x, dcp_dart, base + GATE_T6031_EXT_DCP_DART, fe) ?:
	      gate_x_dart(x, disp_dart, base + GATE_T6031_EXT_DISP_DART, fe) ?:
	      of_changeset_add_prop_string_array(x->cs, mbox, "compatible", mbox_compat,
						 ARRAY_SIZE(mbox_compat)) ?:
	      gate_x_reg(x, mbox, base + GATE_T6031_EXT_MBOX, SZ_16K) ?:
	      gate_x_irqs(x, mbox, GATE_T6031_EXT_MBOX_IRQ, 4) ?:
	      of_changeset_add_prop_string_array(x->cs, mbox, "interrupt-names", mbox_irqs,
						 ARRAY_SIZE(mbox_irqs)) ?:
	      of_changeset_add_prop_u32(x->cs, mbox, "power-domains", fe) ?:
	      of_changeset_add_prop_u32(x->cs, mbox, "#mbox-cells", 0);
	if (ret)
		return ret;

	/*
	 * ATC3 as T6030's: only its core window, so the PHY driver runs it
	 * DisplayPort-only, with the common tunables the boot loader would give
	 * it. Its crossbar sits in atc3_usb, whose parents are atc3_usb_aon and
	 * atc3_common: the two power states the ADT gives atc-phy3.
	 */
	ret = of_changeset_add_prop_string_array(x->cs, phy, "compatible", phy_compat,
						 ARRAY_SIZE(phy_compat)) ?:
	      gate_x_reg(x, phy, GATE_T6031_ATC3, GATE_T6031_ATC3_SIZE) ?:
	      of_changeset_add_prop_string(x->cs, phy, "reg-names", "core") ?:
	      of_changeset_add_prop_u32(x->cs, phy, "#phy-cells", 1) ?:
	      of_changeset_add_prop_u32(x->cs, phy, "#reset-cells", 0) ?:
	      of_changeset_add_prop_bool(x->cs, phy, "orientation-switch") ?:
	      of_changeset_add_prop_bool(x->cs, phy, "mode-switch") ?:
	      of_changeset_add_prop_u32(x->cs, phy, "power-domains", atc_common) ?:
	      of_changeset_add_prop_u32_array(x->cs, phy, "apple,tunable-common-b",
					      x->atc3_common, 3 * x->nr_atc3_common) ?:
	      of_changeset_add_prop_u32(x->cs, phy_ep, "remote-endpoint", ph_dcp_ep) ?:
	      of_changeset_add_prop_string_array(x->cs, xbar, "compatible", xbar_compat,
						 ARRAY_SIZE(xbar_compat)) ?:
	      gate_x_reg(x, xbar, GATE_T6031_ATC3_XBAR, SZ_16K) ?:
	      of_changeset_add_prop_u32(x->cs, xbar, "#mux-control-cells", 1) ?:
	      of_changeset_add_prop_u32(x->cs, xbar, "power-domains", atc_usb);
	if (ret)
		return ret;

	/* dcpext0 and its HDMI output, as the T6030 HDMI port's dcpext. */
	cells[0] = ph_dcp_dart;
	cells[1] = 5;
	ret = of_changeset_add_prop_string_array(x->cs, dcpext, "compatible", dcp_compat,
						 ARRAY_SIZE(dcp_compat)) ?:
	      gate_x_reg(x, dcpext, base + GATE_T6031_EXT_COPROC, SZ_16K) ?:
	      of_changeset_add_prop_string(x->cs, dcpext, "reg-names", "coproc") ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "mboxes", ph_mbox) ?:
	      of_changeset_add_prop_string(x->cs, dcpext, "mbox-names", "mbox") ?:
	      of_changeset_add_prop_u32_array(x->cs, dcpext, "iommus", cells, 2) ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "power-domains", fe) ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "clocks", ph_clk) ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "apple,dcp-index", 1) ?:
	      of_changeset_add_prop_string(x->cs, dcpext, "apple,connector-type", "HDMI-A") ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "#address-cells", 2) ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "#size-cells", 2) ?:
	      of_changeset_add_prop_bool(x->cs, dcpext, "ranges");
	if (ret)
		return ret;
	cells[0] = ph_phy;
	cells[1] = PHY_TYPE_DP;
	ret = of_changeset_add_prop_u32_array(x->cs, dcpext, "phys", cells, 2) ?:
	      of_changeset_add_prop_string(x->cs, dcpext, "phy-names", "dp-phy");
	cells[0] = ph_xbar;
	cells[1] = 0;
	ret = ret ?:
	      of_changeset_add_prop_u32_array(x->cs, dcpext, "mux-controls", cells, 2) ?:
	      of_changeset_add_prop_string(x->cs, dcpext, "mux-control-names", "dp-xbar") ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "mux-index", GATE_T6031_HDMI_XBAR_INPUT) ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "apple,dptx-phy", GATE_T6031_HDMI_DPTX_PHY);
	cells[0] = aop;
	cells[1] = GATE_T6031_HDMI_HPD;
	cells[2] = 0;		/* GPIO_ACTIVE_HIGH */
	ret = ret ?: of_changeset_add_prop_u32_array(x->cs, dcpext, "hdmi-hpd-gpios", cells, 3);
	cells[0] = smc;
	cells[1] = GATE_T6031_HDMI_PWREN;
	ret = ret ?: of_changeset_add_prop_u32_array(x->cs, dcpext, "hdmi-pwren-gpios", cells, 3);
	cells[1] = GATE_T6031_DP2HDMI_PWREN;
	ret = ret ?: of_changeset_add_prop_u32_array(x->cs, dcpext, "dp2hdmi-pwren-gpios",
						     cells, 3);
	ret = ret ?: of_changeset_add_prop_u32(x->cs, dcp_ep, "remote-endpoint", ph_phy_ep);
	if (ret)
		return ret;

	/*
	 * Its firmware, memory and PMP request, handed to the display driver's
	 * native external path with the gate's markers, as on T6030; the
	 * driver checks and maps the memory itself before it runs dcpext0.
	 * The request is not seeded, so that the PMP start, which the panel
	 * needs, is the same as without dcpext0: its entry sets it only once
	 * the PMP has started (apple,request-after-start), and it comes before
	 * dispext0's power, as the parent of dispext0_sys instead of a child of
	 * dispext0_fe. Until the PMP has acknowledged it, the entry provides no
	 * power domain: genpd defers dispext0_sys, _cpu and _fe, and the DARTs,
	 * the mailbox and dcpext0 in them, so nothing powers dispext0 up, and
	 * fw_devlink (on by default) holds their probes, so PMGR writes
	 * dispext0_cpu's floor, when that domain probes, only after it too.
	 */
	cells[0] = ph_disp_dart;
	cells[1] = 4;
	ret = of_changeset_add_prop_u32_array(x->cs, dcpext, "memory-region", region,
					      ARRAY_SIZE(region)) ?:
	      of_changeset_add_prop_string_array(x->cs, dcpext, "memory-region-names", regions,
						 ARRAY_SIZE(regions)) ?:
	      of_changeset_add_prop_u32_array(x->cs, dcpext, "apple,firmware-compat", fw_compat,
					      ARRAY_SIZE(fw_compat)) ?:
	      of_changeset_add_prop_u32_array(x->cs, dcpext, "apple,firmware-version", x->version,
					      x->nr_version) ?:
	      of_changeset_add_prop_string(x->cs, dcpext, "apple,firmware-uuid", x->uuid) ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "apple,pmp-report", entry) ?:
	      of_changeset_add_prop_u32(x->cs, dcpext, "apple,t6030-dcpext-memory-ready", 1) ?:
	      of_changeset_add_prop_u32_array(x->cs, piodma, "iommus", cells, 2) ?:
	      of_changeset_add_prop_u32(x->cs, piodma, "apple,t6030-dispext-handoff", 1) ?:
	      of_changeset_update_prop_string(x->cs, x->entry, "status", "okay") ?:
	      gate_add_bool(x->cs, x->entry, "apple,request-after-start") ?:
	      gate_x_remove(x, x->entry, "power-domains") ?:
	      gate_x_cells(x, x->sys, "power-domains", &entry, 1, true) ?:
	      /* The active floor the display driver checks before it runs dcpext0. */
	      gate_set_u32(x->cs, x->cpu, "apple,min-state", PMGR_PS_ACTIVE) ?:
	      /* As on T6030: no USB consumer holds ATC3 on across system sleep. */
	      gate_add_bool(x->cs, x->atc_common, "apple,always-on");
	if (ret)
		return ret;

	/* Its own display subsystem: scanout through its display DART alone. */
	cells[1] = 0;
	return of_changeset_add_prop_string(x->cs, *subsys, "compatible",
					    "apple,t6031-dispext-subsystem") ?:
	       of_changeset_add_prop_u32_array(x->cs, *subsys, "iommus", cells, 2) ?:
	       of_changeset_add_prop_u32(x->cs, *subsys, "apple,dcps", ph_dcpext);
}

static void __init gate_t6031_ext_put(struct gate_t6031_ext *x)
{
	unsigned int i;

	for (i = 0; i < x->nr_new; i++)
		of_node_put(x->new[i]);
	of_node_put(x->oslog);
	of_node_put(x->text);
	of_node_put(x->entry);
	of_node_put(x->smc);
	of_node_put(x->aop);
	of_node_put(x->atc_usb);
	of_node_put(x->atc_common);
	of_node_put(x->sys);
	of_node_put(x->cpu);
	of_node_put(x->fe);
	of_node_put(x->aic);
	of_node_put(x->rmem);
	of_node_put(x->soc);
	of_node_put(x->root);
}

/* The external display phase; @np are the panel's nodes when @panel. */
static void __init gate_t6031_ext_run(bool panel, struct device_node **np)
{
	struct device_node *subsys = NULL;
	struct gate_adt adt = {};
	struct gate_t6031_ext *x;
	size_t armio;
	unsigned int i;
	int ret;

	if (!gate_t6031_ext)
		return;
	if (!panel) {
		gate_warn("external displays not enabled: apple_t6031_display.ext needs the internal display and the PMP handed over in the same boot\n");
		return;
	}
	if (gate_t6031_ext & ~GATE_T6031_EXT_OUTPUTS) {
		gate_warn("external displays not enabled: apple_t6031_display.ext=%#x, only 0x1 (DCPEXT0, the HDMI port) is described\n",
			  gate_t6031_ext);
		return;
	}
	if (!of_machine_is_compatible("apple,j516c")) {
		gate_t6031_ext_refuse("only the J516C HDMI port is described");
		return;
	}
	if (!gate_t6031_ext_clock || gate_t6031_ext_clock > 2000000000) {
		gate_warn("external displays not enabled: apple_t6031_display.ext_clock=%u Hz is out of range\n",
			  gate_t6031_ext_clock);
		return;
	}
	if (!IS_ENABLED(CONFIG_DRM_APPLE)) {
		gate_t6031_ext_refuse("the display driver is not built");
		return;
	}
	if (IS_ENABLED(CONFIG_VIDEO) && video_firmware_drivers_only()) {
		gate_t6031_ext_refuse("nomodeset is on the command line");
		return;
	}

	x = kzalloc_obj(*x);
	if (!x)
		return;
	ret = gate_adt_load(&adt);
	if (ret) {
		gate_warn("external displays not enabled: no readable boot loader ADT (%d)\n", ret);
		goto out;
	}
	armio = gate_adt_child(&adt, 0, "arm-io");
	if (!armio || gate_adt_u32(&adt, armio, "#address-cells", 0) != 2 ||
	    gate_adt_u32(&adt, armio, "#size-cells", 0) != 2) {
		ret = gate_t6031_ext_refuse("the ADT has no two-cell /arm-io");
		goto out;
	}
	ret = gate_t6031_ext_adt(&adt, armio, &x->fw);
	if (!ret)
		ret = gate_t6031_ext_atc3(&adt, armio, x);
	if (!ret)
		ret = gate_t6031_ext_resolve(x, np);
	if (ret)
		goto out;

	x->cs = gate_cs_alloc();
	if (!x->cs) {
		ret = -ENOMEM;
		goto out;
	}
	x->next_phandle = gate_free_phandle();
	ret = gate_t6031_ext_build(x, &subsys);
	if (!ret)
		ret = of_changeset_apply(x->cs);
	if (ret) {
		/* Nothing applied: the changeset's nodes and properties are dropped. */
		gate_cs_free(x->cs);
		gate_warn("external displays not enabled: changeset failed: %d\n", ret);
		goto out;
	}
	/* Attaching a node, or adding a "phandle", does not set the node's phandle. */
	for (i = 0; i < x->nr_fix; i++)
		x->fix[i]->phandle = x->fix_phandle[i];

	gate_info("DCPEXT0: firmware %s preloaded; DATA %#llx and heap %#llx reserved, OS log %#llx; PMP request %pOF (bit %d) set once the PMP has started, the parent of %pOF\n",
		  x->fw.uuid, x->fw.phys[GATE_SEG_DATA], x->fw.phys[GATE_SEG_HEAP],
		  x->fw.phys[GATE_SEG_OSLOG], x->entry, GATE_T6031_DISPEXT0_REQ, x->sys);
	gate_info("external displays enabled: dcpext0 for the HDMI port, %u nodes added, %u ATC3 common tunables, display clock %u Hz, DRM device %pOF; the PMP starts with the panel's requests alone, and dispext0 powers up once the PMP has acknowledged DISPEXT0\n",
		  x->nr_new, x->nr_atc3_common, gate_t6031_ext_clock, subsys);
out:
	if (ret)
		gate_info("no external displays this boot (%d); the internal display is not affected\n",
			  ret);
	gate_t6031_ext_put(x);
	kvfree(adt.data);
	kfree(x);
}

/*
 * T6031 RTC alarm
 *
 * The PMU that keeps the RTC counter, the primary Chapel (pmic@b on
 * nub_spmi), also has a one-shot alarm that can wake the system. The SMC
 * RTC driver uses it once the device tree describes it as t6031-die0.dtsi
 * does: the PMU's alarm control, compare and mask registers as nvmem cells
 * of pmic@b, the PMU's SPMI peripheral interrupt on nub_spmi, and
 * wakeup-source. The device tree comes from the boot loader, and one built
 * before that description leaves the RTC without its alarm.
 *
 * Unless apple_t6031_rtc.alarm=0, the gate describes the alarm itself then,
 * from this Mac's ADT, in one changeset applied before devices are created.
 * It does so only if the device tree has the RTC as before (its offset cell
 * alone, no interrupt) and the ADT says what the driver expects:
 *  - /arm-io/nub-spmi0 is nub_spmi, at the same address, and an interrupt
 *    controller;
 *  - its one primary PMU has pmic@b's USID and the RTC offset cell the
 *    device tree names, and raises one interrupt on nub-spmi0;
 *  - the alarm control byte also holds the alarm event and has the enable,
 *    event and monitor bits the driver writes (0x40, 0x2 and 0x1), and the
 *    alarm is masked by writing 1 to its mask;
 *  - the control byte, the 32-bit compare value and the mask byte overlap
 *    neither one another nor a cell the device tree has.
 * nub_spmi then becomes an interrupt controller with two interrupt cells,
 * as apple,spmi.yaml requires; pmic@b gets the three cells; and the RTC
 * gets them after its offset cell, the interrupt, rising edge (the only
 * type the controller takes), and wakeup-source. A device tree that
 * describes the alarm already is left as it is. On a J516C the ADT gives
 * the control byte at 0x2000, the compare value at 0x2008, the mask at
 * 0x200e and peripheral interrupt 2.
 */

/* apple_t6031_rtc.alarm=0: leave the RTC alarm undescribed. */
static bool gate_t6031_rtc_alarm __initdata = true;

static int __init gate_setup_t6031_rtc_alarm(char *arg)
{
	return arg ? kstrtobool(arg, &gate_t6031_rtc_alarm) : -EINVAL;
}
early_param("apple_t6031_rtc.alarm", gate_setup_t6031_rtc_alarm);

/* What rtc-macsmc writes to the PMU's alarm registers. */
#define GATE_RTC_CTRL_ALARM_EN		0x40
#define GATE_RTC_CTRL_ALARM_EVENT	0x2
#define GATE_RTC_CTRL_MONITOR		0x1
#define GATE_RTC_ALARM_MASKED		1
/* The SPMI controller's peripheral interrupts, the one type it takes, its registers. */
#define GATE_SPMI_PERIPHERAL_IRQS	256
#define GATE_IRQ_EDGE_RISING		1
#define GATE_SPMI_REGS			0x10000

/* The alarm's control, compare and mask registers, as the RTC names their cells. */
enum { GATE_RTC_CTRL, GATE_RTC_ALARM, GATE_RTC_MASK, GATE_RTC_NR };

static const char *const gate_rtc_node[GATE_RTC_NR] __initconst = {
	"rtc-ctrl", "rtc-alarm", "rtc-alarm-mask",
};
static const u32 gate_rtc_bytes[GATE_RTC_NR] __initconst = { 1, 4, 1 };

#define gate_rtc_info(fmt, ...) pr_info("apple-t6031-rtc: " fmt, ##__VA_ARGS__)
#define gate_rtc_warn(fmt, ...) pr_warn("apple-t6031-rtc: " fmt, ##__VA_ARGS__)

/* What the RTC alarm phase found and checked, and its changeset. */
struct gate_rtc {
	struct device_node *rtc, *offset, *layout, *pmic, *spmi;
	u64 spmi_addr;
	u32 usid;
	u32 offset_reg[2];
	/* nub_spmi is an interrupt controller already. */
	bool spmi_irqc;
	/* From the ADT: the alarm registers, as GATE_RTC_*, and its interrupt. */
	u32 reg[GATE_RTC_NR];
	u32 irq;

	struct of_changeset *cs;
	/* The cells the changeset adds, and the phandles it gives them and nub_spmi. */
	struct device_node *cell[GATE_RTC_NR];
	phandle cell_phandle[GATE_RTC_NR];
	phandle spmi_phandle;
};

static int __init gate_rtc_refuse(const char *why)
{
	gate_rtc_warn("RTC alarm not described: %s\n", why);
	return -EINVAL;
}

/* Whether @bytes bytes at @addr overlap a cell of @layout. */
static bool __init gate_rtc_overlaps(struct device_node *layout, u32 addr, u32 bytes)
{
	struct device_node *cell;
	u32 reg[2];

	for_each_child_of_node(layout, cell) {
		if (of_property_read_u32_array(cell, "reg", reg, 2))
			continue;
		if (addr < (u64)reg[0] + reg[1] && reg[0] < (u64)addr + bytes) {
			of_node_put(cell);
			return true;
		}
	}
	return false;
}

/*
 * The SMC RTC and the PMU cell it reads, as a device tree without the alarm
 * has them. Returns -EALREADY if the device tree describes the alarm.
 */
static int __init gate_rtc_resolve(struct gate_rtc *r)
{
	struct device_node *np;
	struct resource res;
	const char *name;
	unsigned int i;
	u32 cells;

	r->rtc = gate_find_one("apple,smc-rtc");
	if (!r->rtc || !of_device_is_available(r->rtc))
		return gate_rtc_refuse("the device tree has no SMC RTC");
	if (of_property_match_string(r->rtc, "nvmem-cell-names", "rtc_ctrl") >= 0) {
		gate_rtc_info("the device tree describes the RTC alarm\n");
		return -EALREADY;
	}
	if (of_property_count_strings(r->rtc, "nvmem-cell-names") != 1 ||
	    of_property_read_string(r->rtc, "nvmem-cell-names", &name) ||
	    strcmp(name, "rtc_offset") ||
	    of_property_count_u32_elems(r->rtc, "nvmem-cells") != 1 ||
	    of_property_present(r->rtc, "interrupts") ||
	    of_property_present(r->rtc, "interrupts-extended") ||
	    of_property_present(r->rtc, "interrupt-parent") ||
	    of_property_present(r->rtc, "wakeup-source"))
		return gate_rtc_refuse("the SMC RTC has more than its offset cell");

	/* The offset cell, in a fixed layout of a PMU on a T6031 SPMI controller. */
	r->offset = of_parse_phandle(r->rtc, "nvmem-cells", 0);
	r->layout = r->offset ? of_get_parent(r->offset) : NULL;
	r->pmic = r->layout ? of_get_parent(r->layout) : NULL;
	r->spmi = r->pmic ? of_get_parent(r->pmic) : NULL;
	if (!r->spmi || !r->offset->phandle ||
	    of_property_count_u32_elems(r->offset, "reg") != 2 ||
	    of_property_read_u32_array(r->offset, "reg", r->offset_reg, 2) ||
	    !of_node_name_eq(r->layout, "nvmem-layout") ||
	    !of_device_is_compatible(r->layout, "fixed-layout") ||
	    of_property_read_u32(r->layout, "#address-cells", &cells) || cells != 1 ||
	    of_property_read_u32(r->layout, "#size-cells", &cells) || cells != 1 ||
	    !of_device_is_compatible(r->pmic, "apple,spmi-nvmem") ||
	    !of_device_is_available(r->pmic) ||
	    of_property_read_u32_index(r->pmic, "reg", 0, &r->usid) ||
	    !of_device_is_compatible(r->spmi, "apple,t6031-spmi") ||
	    !of_device_is_available(r->spmi) ||
	    /* The controller has peripheral interrupts only with its own. */
	    !of_property_present(r->spmi, "interrupts") ||
	    of_address_to_resource(r->spmi, 0, &res))
		return gate_rtc_refuse("the RTC offset cell is not in a fixed layout of a PMU on a T6031 SPMI controller with an interrupt");
	r->spmi_addr = res.start;

	r->spmi_irqc = of_property_read_bool(r->spmi, "interrupt-controller");
	if (r->spmi_irqc ? of_property_read_u32(r->spmi, "#interrupt-cells", &cells) || cells != 2 :
			   of_property_present(r->spmi, "#interrupt-cells"))
		return gate_rtc_refuse("the SPMI controller has other interrupt cells");

	for (i = 0; i < GATE_RTC_NR; i++) {
		np = of_get_child_by_name(r->layout, gate_rtc_node[i]);
		of_node_put(np);
		if (np)
			return gate_rtc_refuse("the PMU has an alarm cell already");
	}
	return 0;
}

/* The ADT's one primary PMU below @spmi with USID @usid, or 0. */
static size_t __init gate_rtc_adt_pmu(const struct gate_adt *adt, size_t spmi, u32 usid)
{
	u32 props = get_unaligned_le32(adt->data + spmi);
	u32 children = get_unaligned_le32(adt->data + spmi + 4);
	size_t off = spmi + 8, found = 0;
	u32 i;

	for (i = 0; i < props; i++)
		off += 36 + ALIGN(get_unaligned_le32(adt->data + off + 32) & 0x7fffffff, 4);
	for (i = 0; i < children; i++) {
		if (gate_adt_u32(adt, off, "is-primary", 0) == 1 &&
		    gate_adt_u32(adt, off, "reg", 0) == usid) {
			if (found)
				return 0;
			found = off;
		}
		off = gate_adt_node_end(adt, off, 1);
		if (!off)
			return 0;
	}
	return found;
}

/* The alarm, as the ADT gives it, checked against the driver and the device tree. */
static int __init gate_rtc_adt(const struct gate_adt *adt, struct gate_rtc *r)
{
	size_t armio, spmi, pmu = 0;
	const u8 *irq = NULL, *parent = NULL;
	u32 len = 0, plen = 0, ph;
	u64 addr, size;
	unsigned int i, j;

	armio = gate_adt_child(adt, 0, "arm-io");
	if (!armio || gate_adt_u32(adt, armio, "#address-cells", 0) != 2 ||
	    gate_adt_u32(adt, armio, "#size-cells", 0) != 2)
		return gate_rtc_refuse("the ADT has no two-cell /arm-io");
	spmi = gate_adt_child(adt, armio, "nub-spmi0");
	ph = spmi ? gate_adt_u32(adt, spmi, "AAPL,phandle", 0) : U32_MAX;
	if (!spmi || gate_adt_reg(adt, armio, spmi, 0, &addr, &size) || addr != r->spmi_addr ||
	    !gate_adt_prop(adt, spmi, "interrupt-controller", &len) || ph == U32_MAX)
		return gate_rtc_refuse("the ADT's nub-spmi0 is not this SPMI controller as an interrupt controller");

	pmu = gate_rtc_adt_pmu(adt, spmi, r->usid);
	if (pmu) {
		irq = gate_adt_prop(adt, pmu, "interrupts", &len);
		parent = gate_adt_prop(adt, pmu, "interrupt-parent", &plen);
	}
	if (!irq || len != 4 || !parent || plen != 4 || get_unaligned_le32(parent) != ph)
		return gate_rtc_refuse("the ADT has no primary PMU at this USID with one interrupt on nub-spmi0");
	r->irq = get_unaligned_le32(irq);
	if (r->irq >= GATE_SPMI_PERIPHERAL_IRQS)
		return gate_rtc_refuse("the PMU's interrupt is not a peripheral interrupt");

	r->reg[GATE_RTC_CTRL] = gate_adt_u32(adt, pmu, "info-rtc_alarm_ctrl", 0);
	r->reg[GATE_RTC_ALARM] = gate_adt_u32(adt, pmu, "info-rtc_alarm_offset", 0);
	r->reg[GATE_RTC_MASK] = gate_adt_u32(adt, pmu, "info-rtc_irq_mask_offset", 0);
	if (gate_adt_u32(adt, pmu, "info-rtc_alarm_event", 0) != r->reg[GATE_RTC_CTRL] ||
	    gate_adt_u32(adt, pmu, "info-rtc_alarm_ctrl_en_mask", 0) != GATE_RTC_CTRL_ALARM_EN ||
	    gate_adt_u32(adt, pmu, "info-rtc_alarm_event_mask", 0) != GATE_RTC_CTRL_ALARM_EVENT ||
	    gate_adt_u32(adt, pmu, "info-rtc_alarm_monitor_mask", 0) != GATE_RTC_CTRL_MONITOR ||
	    gate_adt_u32(adt, pmu, "info-rtc_alarm_mask", 0) != GATE_RTC_ALARM_MASKED)
		return gate_rtc_refuse("the PMU's alarm is not the one the RTC driver programs");
	if (gate_adt_u32(adt, pmu, "info-clock_offset", 0) != r->offset_reg[0] ||
	    gate_adt_u32(adt, pmu, "info-clock_offset", 1) != r->offset_reg[1])
		return gate_rtc_refuse("the PMU's RTC offset is not the cell the device tree names");
	for (i = 0; i < GATE_RTC_NR; i++) {
		if ((u64)r->reg[i] + gate_rtc_bytes[i] > GATE_SPMI_REGS ||
		    gate_rtc_overlaps(r->layout, r->reg[i], gate_rtc_bytes[i]))
			return gate_rtc_refuse("an alarm register is out of range or in a cell already");
		for (j = 0; j < i; j++)
			if (r->reg[i] < (u64)r->reg[j] + gate_rtc_bytes[j] &&
			    r->reg[j] < (u64)r->reg[i] + gate_rtc_bytes[i])
				return gate_rtc_refuse("the alarm registers overlap");
	}
	return 0;
}

/* Sets @name of @np to @len bytes of @value, adding it if it is missing. */
static int __init gate_rtc_set(struct of_changeset *cs, struct device_node *np,
			       const char *name, const void *value, size_t len)
{
	/* Kept for good once applied: the live tree refers to it. */
	struct property *prop = kzalloc_obj(*prop);
	int ret = -ENOMEM;

	if (prop) {
		prop->name = kstrdup(name, GFP_KERNEL);
		prop->value = kmemdup(value, len, GFP_KERNEL);
		prop->length = len;
		if (prop->name && prop->value)
			ret = of_changeset_update_property(cs, np, prop);
		if (ret) {
			kfree(prop->name);
			kfree(prop->value);
			kfree(prop);
		}
	}
	return ret;
}

/* The alarm's cells, interrupt and wakeup-source, in the changeset. */
static int __init gate_rtc_build(struct gate_rtc *r)
{
	static const char names[] = "rtc_offset\0rtc_ctrl\0rtc_alarm\0rtc_alarm_mask";
	phandle next = gate_free_phandle();
	__be32 cells[1 + GATE_RTC_NR];
	u32 reg[2], irq[3];
	char name[32];
	unsigned int i;
	int ret = 0;

	r->spmi_phandle = r->spmi->phandle;
	if (!r->spmi_phandle) {
		r->spmi_phandle = next++;
		ret = of_changeset_add_prop_u32(r->cs, r->spmi, "phandle", r->spmi_phandle);
	}
	if (!ret && !r->spmi_irqc)
		ret = of_changeset_add_prop_bool(r->cs, r->spmi, "interrupt-controller") ?:
		      of_changeset_add_prop_u32(r->cs, r->spmi, "#interrupt-cells", 2);

	cells[0] = cpu_to_be32(r->offset->phandle);
	for (i = 0; i < GATE_RTC_NR && !ret; i++) {
		snprintf(name, sizeof(name), "%s@%x", gate_rtc_node[i], r->reg[i]);
		r->cell[i] = of_changeset_create_node(r->cs, r->layout, name);
		if (!r->cell[i])
			return -ENOMEM;
		r->cell_phandle[i] = next++;
		cells[1 + i] = cpu_to_be32(r->cell_phandle[i]);
		reg[0] = r->reg[i];
		reg[1] = gate_rtc_bytes[i];
		ret = of_changeset_add_prop_u32_array(r->cs, r->cell[i], "reg", reg, 2) ?:
		      of_changeset_add_prop_u32(r->cs, r->cell[i], "phandle", r->cell_phandle[i]);
	}
	if (ret)
		return ret;

	irq[0] = r->spmi_phandle;
	irq[1] = r->irq;
	irq[2] = GATE_IRQ_EDGE_RISING;
	return gate_rtc_set(r->cs, r->rtc, "nvmem-cells", cells, sizeof(cells)) ?:
	       gate_rtc_set(r->cs, r->rtc, "nvmem-cell-names", names, sizeof(names)) ?:
	       of_changeset_add_prop_u32_array(r->cs, r->rtc, "interrupts-extended", irq, 3) ?:
	       of_changeset_add_prop_bool(r->cs, r->rtc, "wakeup-source");
}

/* Checks the alarm against the ADT and describes it; nothing changes on failure. */
static int __init gate_rtc_apply(const struct gate_adt *adt, struct gate_rtc *r)
{
	unsigned int i;
	int ret;

	ret = gate_rtc_adt(adt, r);
	if (ret)
		return ret;
	r->cs = gate_cs_alloc();
	if (!r->cs)
		return -ENOMEM;
	ret = gate_rtc_build(r);
	if (!ret)
		ret = of_changeset_apply(r->cs);
	if (ret) {
		/* Nothing applied: the changeset's nodes and properties are dropped. */
		gate_cs_free(r->cs);
		r->cs = NULL;
		gate_rtc_warn("RTC alarm not described: changeset failed: %d\n", ret);
		return ret;
	}
	/* Attaching a node, or adding a "phandle", does not set the node's phandle. */
	r->spmi->phandle = r->spmi_phandle;
	for (i = 0; i < GATE_RTC_NR; i++)
		r->cell[i]->phandle = r->cell_phandle[i];

	gate_rtc_info("RTC alarm described from the ADT: %pOFP, %pOFP and %pOFP in %pOF, interrupt %u of %pOF (rising edge), wakeup source\n",
		      r->cell[GATE_RTC_CTRL], r->cell[GATE_RTC_ALARM], r->cell[GATE_RTC_MASK],
		      r->layout, r->irq, r->spmi);
	return 0;
}

static void __init gate_rtc_put(struct gate_rtc *r)
{
	unsigned int i;

	for (i = 0; i < GATE_RTC_NR; i++)
		of_node_put(r->cell[i]);
	of_node_put(r->spmi);
	of_node_put(r->pmic);
	of_node_put(r->layout);
	of_node_put(r->offset);
	of_node_put(r->rtc);
}

static void __init gate_t6031_rtc_run(void)
{
	struct gate_adt adt = {};
	struct gate_rtc r = {};
	int ret;

	/* Nothing would use it. */
	if (!IS_ENABLED(CONFIG_RTC_DRV_MACSMC))
		return;
	if (!gate_t6031_rtc_alarm) {
		gate_rtc_info("RTC alarm not described: apple_t6031_rtc.alarm=0\n");
		return;
	}
	ret = gate_rtc_resolve(&r);
	if (!ret) {
		ret = gate_adt_load(&adt);
		if (ret)
			gate_rtc_warn("RTC alarm not described: no readable boot loader ADT (%d)\n",
				      ret);
	}
	if (!ret)
		gate_rtc_apply(&adt, &r);
	gate_rtc_put(&r);
	kvfree(adt.data);
}

static void __init gate_run(void)
{
	struct device_node *np[GATE_NR_NODES] = {};
	struct device_node *ps[PMP_PS_NR] = {};
	struct device_node *aic = NULL;
	bool panel = false;
	int i;

	if (gate_soc->describe)
		gate_describe();

	if (gate_soc == &gate_t6031) {
		const char *board = gate_t6031_default_board();

		gate_t6031_requested = gate_t6031_wanted(gate_t6031_switch, !!board);
		if (gate_t6031_switch < 0 && board)
			gate_info("native display on by default on %s; apple_t6031_display.enable=0 keeps the boot framebuffer\n",
				  board);
	}

	if (gate_soc == &gate_t6034) {
		if (!gate_t6034_default_board()) {
			gate_info("unsupported board; native display stays disabled\n");
			return;
		}
		gate_t6034_requested = gate_t6034_wanted(gate_t6034_switch);
		if (gate_t6034_switch < 0)
			gate_info("native display on by default on J514M; apple_t6034_display.enable=0 keeps the boot framebuffer\n");
	}

	if (!*gate_soc->requested) {
		if (gate_soc->opt_in && gate_t6031_switch < 0)
			gate_info("native display not requested (%s is not on the command line), display stays on the boot framebuffer\n",
				  gate_soc->opt_in);
		else
			gate_info("disabled on the command line, display stays on the boot framebuffer\n");
		if (gate_soc->dcpext)
			gate_dcpext("the display gate is disabled");
		if (gate_soc == &gate_t6031)
			gate_t6031_ext_run(false, NULL);
		return;
	}

	if (!gate_resolve(np) && !gate_pmp_resolve(np, ps, &aic) && !gate_apply(np)) {
		if (gate_pmp_apply(np[GATE_DCP], ps, aic))
			gate_revert();
		else
			panel = true;
	}
	/* External processors need the panel's PMP; without it they stay off. */
	if (gate_soc->dcpext)
		gate_dcpext(panel ? NULL : "the internal display was not handed over");
	if (gate_soc == &gate_t8122)
		gate_t8122_dcpext(panel);
	if (gate_soc == &gate_t6031)
		gate_t6031_ext_run(panel, np);
	gate_reuse_put();
	of_node_put(aic);
	for (i = 0; i < PMP_PS_NR; i++)
		of_node_put(ps[i]);
	for (i = 0; i < GATE_NR_NODES; i++)
		of_node_put(np[i]);
}

/* Runs before of_platform_default_populate_init() at arch_initcall_sync. */
static int __init apple_t6030_display_gate(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gate_socs) && !gate_soc; i++)
		if (of_machine_is_compatible(gate_socs[i]->machine))
			gate_soc = gate_socs[i];
	if (!gate_soc)
		return 0;

	/* The exact 25G83 tree (J613; J615 experimental) has its own firmware and PMP admission gate. */
	if (of_machine_is_compatible("apple,j613") || of_machine_is_compatible("apple,j615")) {
		struct device_node *dcp = of_find_node_by_path("/soc/dcp@28ec00000");
		bool native = dcp && of_property_present(dcp, "apple,j613-25g83-profile");

		of_node_put(dcp);
		if (native)
			return 0;
	}

	gate_run();
	/* Not a display matter, but the same boot loader handoff and ADT. */
	if (gate_soc == &gate_t6031)
		gate_t6031_rtc_run();
	return 0;
}
arch_initcall(apple_t6030_display_gate);

#if IS_ENABLED(CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST)
#include "t6030-display-gate-test.c"
#endif
