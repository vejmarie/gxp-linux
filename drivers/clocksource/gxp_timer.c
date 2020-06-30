// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched_clock.h>

#include <asm/irq.h>

#define TIMER0_FREQ 1000000
#define TIMER1_FREQ 1000000

#define MASK_TCS_ENABLE		0x01
#define MASK_TCS_PERIOD		0x02
#define MASK_TCS_RELOAD		0x04
#define MASK_TCS_TC		0x80

struct gxp_timer {
	void __iomem *counter;
	void __iomem *control;
	struct clock_event_device evt;
	struct irqaction act;
};

static void __iomem *system_clock __read_mostly;

static u64 notrace gxp_sched_read(void)
{
	return readl_relaxed(system_clock);
}

static int gxp_time_set_next_event(unsigned long event,
					struct clock_event_device *evt_dev)
{
	struct gxp_timer *timer = container_of(evt_dev, struct gxp_timer, evt);
	// clear TC by write 1 and disable timer int and counting
	writeb_relaxed(MASK_TCS_TC, timer->control);
	// update counter value
	writel_relaxed(event, timer->counter);
	// enable timer counting and int
	writeb_relaxed(MASK_TCS_TC|MASK_TCS_ENABLE, timer->control);

	return 0;
}

static irqreturn_t gxp_time_interrupt(int irq, void *dev_id)
{
	struct gxp_timer *timer = dev_id;
	void (*event_handler)(struct clock_event_device *timer);

	if (readb_relaxed(timer->control) & MASK_TCS_TC) {
		writeb_relaxed(MASK_TCS_TC, timer->control);

		event_handler = READ_ONCE(timer->evt.event_handler);
		if (event_handler)
			event_handler(&timer->evt);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int __init gxp_timer_init(struct device_node *node)
{
	void __iomem *base_counter;
	void __iomem *base_control;
	u32 freq;
	int irq;
	struct gxp_timer *timer;

	base_counter = of_iomap(node, 0);
	if (!base_counter)
		panic("Can't remap counter registers");

	base_control = of_iomap(node, 1);
	if (!base_control)
		panic("Can't remap control registers");

	system_clock = of_iomap(node, 2);
	if (!system_clock)
		panic("Can't remap control registers");

	if (of_property_read_u32(node, "clock-frequency", &freq))
		panic("Can't read clock-frequency");

	sched_clock_register(gxp_sched_read, 32, freq);
	clocksource_mmio_init(system_clock, node->name, freq,
				300, 32, clocksource_mmio_readl_up);

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("Can't parse IRQ");

	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (!timer)
		return -ENOMEM;


	timer->counter = base_counter;
	timer->control = base_control;
	timer->evt.name = node->name;
	timer->evt.rating = 300;
	timer->evt.features = CLOCK_EVT_FEAT_ONESHOT;
	timer->evt.set_next_event = gxp_time_set_next_event;
	timer->evt.cpumask = cpumask_of(0);
	timer->act.name = node->name;
	timer->act.flags = IRQF_TIMER | IRQF_SHARED;
	timer->act.dev_id = timer;
	timer->act.handler = gxp_time_interrupt;

	if (setup_irq(irq, &timer->act))
		panic("Can't set up timer IRQ\n");

	clockevents_config_and_register(&timer->evt, TIMER0_FREQ,
					0xf, 0xffffffff);

	pr_info("gxp: system timer (irq = %d)\n", irq);
	return 0;
}
TIMER_OF_DECLARE(gxp, "hpe,gxp-timer", gxp_timer_init);
