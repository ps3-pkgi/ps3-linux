/*
 * Copyright (C) 2012 by Alan Stern
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* This file is part of ehci-hcd.c */

/*-------------------------------------------------------------------------*/

/* Set a bit in the USBCMD register */
static void ehci_set_command_bit(struct ehci_hcd *ehci, u32 bit)
{
	ehci->command |= bit;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	/* unblock posted write */
	ehci_readl(ehci, &ehci->regs->command);
}

/* Clear a bit in the USBCMD register */
static void ehci_clear_command_bit(struct ehci_hcd *ehci, u32 bit)
{
	ehci->command &= ~bit;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	/* unblock posted write */
	ehci_readl(ehci, &ehci->regs->command);
}

/*-------------------------------------------------------------------------*/

/*
 * EHCI timer support...  Now using hrtimers.
 *
 * Lots of different events are triggered from ehci->hrtimer.  Whenever
 * the timer routine runs, it checks each possible event; events that are
 * currently enabled and whose expiration time has passed get handled.
 * The set of enabled events is stored as a collection of bitflags in
 * ehci->enabled_hrtimer_events, and they are numbered in order of
 * increasing delay values (ranging between 1 ms and 100 ms).
 *
 * Rather than implementing a sorted list or tree of all pending events,
 * we keep track only of the lowest-numbered pending event, in
 * ehci->next_hrtimer_event.  Whenever ehci->hrtimer gets restarted, its
 * expiration time is set to the timeout value for this event.
 *
 * As a result, events might not get handled right away; the actual delay
 * could be anywhere up to twice the requested delay.  This doesn't
 * matter, because none of the events are especially time-critical.  The
 * ones that matter most all have a delay of 1 ms, so they will be
 * handled after 2 ms at most, which is okay.  In addition to this, we
 * allow for an expiration range of 1/2 ms.
 */

/*
 * Delay lengths for the hrtimer event types.
 * Keep this sorted by delay lengths, and in the same order as
 * the events types indexed by enum ehci_hrtimer_event in ehci.h.
 */
static unsigned ehci_event_delays_ns[] = {
	1 * NSEC_PER_MSEC,	/* EHCI_HRTIMER_POLL_PSS */
	10 * NSEC_PER_MSEC,	/* EHCI_DISABLE_PERIODIC */
};

/* Enable a pending hrtimer event */
static void ehci_enable_event(struct ehci_hcd *ehci, unsigned event,
		ktime_t *timeout, bool resched)
{
	if (resched)
		*timeout = ktime_add(ktime_get(),
				ktime_set(0, ehci_event_delays_ns[event]));
	ehci->enabled_hrtimer_events |= (1 << event);

	/* Track only the lowest-numbered event */
	if (event < ehci->next_hrtimer_event) {
		ehci->next_hrtimer_event = event;
		hrtimer_start_range_ns(&ehci->hrtimer, *timeout,
				NSEC_PER_MSEC / 2, HRTIMER_MODE_ABS);
	}
}


/* Poll the STS_PSS status bit */
static void ehci_poll_PSS(struct ehci_hcd *ehci, ktime_t *now)
{
	u32	status;

	/* Don't enable anything if the controller isn't running (e.g., died) */
	if (!(ehci->command & CMD_RUN))
		return;

	/*
	 * When the periodic schedule stops, restart it.  If it takes
	 * too long to stop then give up and restart it anyway, otherwise
	 * continue polling.
	 */
	status = ehci_readl(ehci, &ehci->regs->status);
	if (unlikely(now && (status & STS_PSS))) {
		if (now->tv64 >= ehci->PSS_giveup_time.tv64) {
			ehci_warn(ehci, "Waited too long for the periodic schedule to stop, giving up\n");
			status = 0;
		}
	}

	if (!(status & STS_PSS))
		ehci_set_command_bit(ehci, CMD_PSE);

	else		/* Poll again later */
		ehci_enable_event(ehci, EHCI_HRTIMER_POLL_PSS,
				&ehci->periodic_event_time, true);
}

/* Disable the periodic schedule */
static void ehci_turn_off_PSE(struct ehci_hcd *ehci)
{
	ehci_clear_command_bit(ehci, CMD_PSE);
	free_cached_lists(ehci);

	/* Allow up to 20 ms for the schedule to actually stop */
	ehci->PSS_giveup_time = ktime_add(ktime_get(),
			ktime_set(0, 20 * NSEC_PER_MSEC));
}


static enum hrtimer_restart ehci_hrtimer_func(struct hrtimer *t)
{
	struct ehci_hcd	*ehci = container_of(t, struct ehci_hcd, hrtimer);
	ktime_t		now = ktime_get();
	unsigned	events;
	unsigned long	flags;

	spin_lock_irqsave(&ehci->lock, flags);

	events = ehci->enabled_hrtimer_events;
	ehci->enabled_hrtimer_events = 0;
	ehci->next_hrtimer_event = EHCI_HRTIMER_NO_EVENT;

	/*
	 * Check each pending event.  If its time has expired, handle
	 * the event; otherwise re-enable it.
	 */
	if (events & BIT(EHCI_HRTIMER_POLL_PSS)) {
		if (now.tv64 >= ehci->periodic_event_time.tv64)
			ehci_poll_PSS(ehci, &now);
		else
			ehci_enable_event(ehci, EHCI_HRTIMER_POLL_PSS,
					&ehci->periodic_event_time, false);
	}

	if (events & BIT(EHCI_HRTIMER_DISABLE_PERIODIC)) {
		if (now.tv64 >= ehci->periodic_event_time.tv64)
			ehci_turn_off_PSE(ehci);
		else
			ehci_enable_event(ehci, EHCI_HRTIMER_DISABLE_PERIODIC,
					&ehci->periodic_event_time, false);
	}

	spin_unlock_irqrestore(&ehci->lock, flags);
	return HRTIMER_NORESTART;
}
